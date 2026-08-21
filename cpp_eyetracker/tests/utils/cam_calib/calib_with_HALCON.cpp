// calib_with_HALCON.cpp — 标定图片采集 + TCP 传输 + 触发 HALCON 标定链 (正式版)
// Master: 相机预览 + 从 Slave 拉取缺失图片 (TCP, 独立数据端口) + 启动 calib_cam_chain
// Slave:  相机预览 + 响应 Master 的 LIST/XFER 请求
// ====================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
#endif

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <set>
#include <map>
#include <system_error>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== Globals ==================
atomic<bool> global_running{true};
bool g_is_master = false, g_enable_net_sync = false;
string g_save_dir;  // calib_save_dir/{day_id}
string g_xml_dir;   // 标定 XML 输出目录 = cam_calib.yaml: calib_save_dir/{input_day_id}/output
atomic<bool> g_intr_mode{false};  // 内参标定模式 ('i' 切换; master 通过 MODE: 命令同步给 slave)
string g_intr_root;               // 内参图片根目录 = calib_save_dir/{participant_id}/pictures
string g_intr_xml_dir;            // 内参标定 XML 输出目录 = calib_save_dir/{participant_id}/output
int g_win_w=1600, g_win_h=800, g_left_w, g_right_x, g_right_w, g_thumb_w, g_thumb_h;
atomic<int> g_enlarged_cam{-1};
// UI 模式: CAPTURE = 正常采集界面; CALIBRATING = 标定期间全屏状态界面 (不渲染缩略图, 降低 CPU)
enum class UiMode { CAPTURE, CALIBRATING };
UiMode g_ui_mode = UiMode::CAPTURE;
chrono::steady_clock::time_point g_calib_start;
SOCKET g_ctrl_sock = INVALID_SOCKET;  // TCP control channel (text commands)
SOCKET g_listen_sock = INVALID_SOCKET;
int g_ctrl_port = 0;                 // control port; data port = g_ctrl_port + 1
string g_master_ip;                  // Slave needs this to connect to data port
atomic<bool> g_fault_active{false}; atomic<int> g_faulty_cam{-1};
atomic<bool> g_fault_on_master{false};
chrono::steady_clock::time_point g_fault_time, g_ready_time;
atomic<int> g_capture_count{-1}; int g_undo_count=0;   // 逻辑拍摄数 = 下一张后缀 (z 回退递减, 下次拍摄覆盖)
#ifdef _WIN32
PROCESS_INFORMATION g_halcon_pi{};       // calib_cam_chain 子进程 (外参模式 t 键传输完成后触发)
PROCESS_INFORMATION g_intrinsics_pi{};   // calib_cam_intrinsics 子进程 (内参模式 t 键传输完成后触发)
#endif
string g_calib_title="cam_calib_chain is running";   // CALIBRATING 界面副标题 (启动时设置)

// recvLine leftover: catches data past \n when multiple lines arrive in one TCP segment
thread_local string g_tcp_leftover;

// ================== TCP helpers ==================
bool recvLine(SOCKET s, string& line, int timeout_ms=3000) {
    DWORD to=timeout_ms; setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,(const char*)&to,sizeof(to));
    size_t nl=g_tcp_leftover.find('\n');
    if(nl!=string::npos){line=g_tcp_leftover.substr(0,nl);if(!line.empty()&&line.back()=='\r')line.pop_back();g_tcp_leftover=g_tcp_leftover.substr(nl+1);return true;}
    char buf[256]; auto dl=chrono::steady_clock::now()+chrono::milliseconds(timeout_ms);
    while(chrono::steady_clock::now()<dl){int n=recv(s,buf,sizeof(buf)-1,0);if(n<=0)return false;buf[n]='\0';g_tcp_leftover+=buf;
        nl=g_tcp_leftover.find('\n');if(nl!=string::npos){line=g_tcp_leftover.substr(0,nl);if(!line.empty()&&line.back()=='\r')line.pop_back();g_tcp_leftover=g_tcp_leftover.substr(nl+1);return true;}}
    return false;
}

bool sendExact(SOCKET s,const void* buf,size_t n){size_t sent=0;while(sent<n){int r=send(s,(const char*)buf+sent,(int)(n-sent),0);if(r<=0)return false;sent+=r;}return true;}
bool sendLine(SOCKET s,const string& m){string d=m+"\n";return sendExact(s,d.c_str(),d.length());}
bool recvExact(SOCKET s,void* buf,size_t n){size_t got=0;while(got<n){int r=recv(s,(char*)buf+got,(int)(n-got),0);if(r<=0)return false;got+=r;}return true;}

// ================== HALCON 标定子进程 ==================
#ifdef _WIN32
// 通用: 启动同目录下的 HALCON 程序 (已在运行则跳过). 返回 true = 子进程正在运行
bool launchChild(const string& exe_name,const string& display,PROCESS_INFORMATION& pi){
    if (pi.hProcess) {
        DWORD ec = 0;
        if (GetExitCodeProcess(pi.hProcess, &ec) && ec == STILL_ACTIVE) {
            cout << "[HALCON] " << display << " already running. Please wait." << endl;
            return true;
        }
        CloseHandle(pi.hProcess); CloseHandle(pi.hThread);
        ZeroMemory(&pi, sizeof(pi));
    }
    char self_path[MAX_PATH];
    GetModuleFileNameA(NULL, self_path, MAX_PATH);
    fs::path exe = fs::path(self_path).parent_path() / exe_name;
    if (!fs::exists(exe)) {
        cerr << "[HALCON] Not found: " << exe.string()
             << " (需设置 HALCONROOT 并编译 " << display << ")" << endl;
        return false;
    }
    STARTUPINFOA si{}; si.cb = sizeof(si);
    if (CreateProcessA(exe.string().c_str(), NULL, NULL, NULL, FALSE, 0,
                       NULL, NULL, &si, &pi)) {
        cout << "[HALCON] Launched " << display << ".exe" << endl;
        return true;
    } else {
        cerr << "[HALCON] Launch failed (error " << GetLastError() << "): "
             << exe.string() << endl;
        return false;
    }
}
bool launchHalconChain(){return launchChild("cam_calib_chain.exe","cam_calib_chain",g_halcon_pi);}
bool launchIntrinsicsCalib(){return launchChild("calib_cam_intrinsics.exe","calib_cam_intrinsics",g_intrinsics_pi);}
// 启动 viz_calib_chain.py 可视化 (参数 = 目录 ID: day_id 或 participant_id)
void launchViz(const string& viz_id){
    // 本机默认 Qt 后端不可用, 强制 TkAgg (与 DEV_GUIDE 一致: $env:MPLBACKEND="TkAgg"), 子进程继承
    SetEnvironmentVariableA("MPLBACKEND","TkAgg");
    fs::path script=fs::path(__FILE__).parent_path()/"viz_calib_chain.py";
    string cmd="python \""+script.string()+"\" "+viz_id;
    STARTUPINFOA si{};si.cb=sizeof(si);
    PROCESS_INFORMATION pi{};
    if(CreateProcessA(NULL,cmd.data(),NULL,NULL,FALSE,0,NULL,NULL,&si,&pi)){
        CloseHandle(pi.hProcess);CloseHandle(pi.hThread);
        cout<<"[Viz] Launched viz_calib_chain.py ("<<viz_id<<")"<<endl;
    }else{
        cerr<<"[Viz] Launch failed (error "<<GetLastError()<<"): "<<cmd<<endl;
    }
}
#else
bool launchHalconChain() { cerr << "[HALCON] Not supported on this platform." << endl; return false; }
bool launchIntrinsicsCalib() { cerr << "[HALCON] Not supported on this platform." << endl; return false; }
void launchViz(const string&) { cerr << "[Viz] Not supported on this platform." << endl; }
#endif

// ================== Camera ==================
struct CameraContext {string sn; BaslerCamera cam{""}; bool is_mono=true; string status_msg="Initializing"; thread capture_thread,copy_thread; atomic<bool> running{true};
    cv::Mat latest_frame; FrameMeta latest_meta; mutex frame_mtx;
    queue<pair<cv::Mat,FrameMeta>> copy_queue; mutex copy_mtx; condition_variable copy_cv;  // [Fix] 队列存已拷贝 Mat, 不持有采集卡缓冲
    atomic<int64_t> last_block_id{-1}; atomic<chrono::steady_clock::time_point> last_frame_time{chrono::steady_clock::now()}; atomic<bool> has_streamed{false};
    explicit CameraContext(string s):sn(s),cam(s){}};
vector<shared_ptr<CameraContext>> cam_ctxs;

void copyWorker(shared_ptr<CameraContext> ctx){while(ctx->running){pair<cv::Mat,FrameMeta> t;
    {unique_lock<mutex> lk(ctx->copy_mtx);ctx->copy_cv.wait(lk,[&]{return !ctx->copy_queue.empty()||!ctx->running;});if(!ctx->running&&ctx->copy_queue.empty())break;t=ctx->copy_queue.front();ctx->copy_queue.pop();}
    {lock_guard<mutex> lk(ctx->frame_mtx);ctx->latest_frame=t.first;ctx->latest_meta=t.second;}
    ctx->last_block_id.store(t.second.blockID,memory_order_relaxed);ctx->last_frame_time.store(chrono::steady_clock::now(),memory_order_relaxed);ctx->has_streamed.store(true,memory_order_relaxed);}}

void captureWorker(shared_ptr<CameraContext> ctx,double fps,double gain,double gamma,double exp,double me){
    if(!ctx->cam.open(TriggerMode::Software)){ctx->status_msg="OPEN FAILED";return;}ctx->is_mono=ctx->cam.isMono();if(ctx->is_mono&&me>1.0){exp*=me;fps/=me;}
    try{ctx->cam.setFrameRate(fps);ctx->cam.setGain(gain);ctx->cam.setGamma(gamma);ctx->cam.setExposureTime(exp);}catch(...){}
    // 回调内同步拷贝: 采集卡缓冲只在回调期间被持有, 随后立即归还驱动
    ctx->cam.setFrameCallback([ctx](const Pylon::CBaslerUniversalGrabResultPtr& p,FrameMeta m){
        cv::Mat tmp(p->GetHeight(),p->GetWidth(),CV_8UC1,p->GetBuffer());
        cv::Mat clone_img=tmp.clone();
        lock_guard<mutex> lk(ctx->copy_mtx);
        if(ctx->copy_queue.size()<2){ctx->copy_queue.push({clone_img,m});ctx->copy_cv.notify_one();}
    });
    if(!ctx->cam.start()){ctx->status_msg="START FAILED";return;}ctx->status_msg="STREAMING";while(ctx->running)this_thread::sleep_for(chrono::milliseconds(50));ctx->cam.close();}

int getNextCounter(const string& dir){int mx=-1;if(!fs::exists(dir))return 0;for(auto& e:fs::directory_iterator(dir)){if(e.path().extension()==".jpg"){try{string s=e.path().stem().string();size_t u=s.find_last_of('_');if(u!=string::npos)mx=max(mx,stoi(s.substr(u+1)));}catch(...){}}}return mx+1;}
// 扫描图片, 返回相对根目录的键 (不含扩展名). 外参模式: calib_cam_SN_NNN; 内参模式: SN/calib_cam_SN_NNN
set<string> scanFiles(const string& root,bool nested){set<string> r;if(!fs::exists(root))return r;
    if(!nested){for(auto& e:fs::directory_iterator(root)){if(e.path().extension()==".jpg"){string s=e.path().stem().string();if(s.rfind("calib_cam_",0)==0)r.insert(s);}}}
    else{for(auto& e:fs::directory_iterator(root)){if(!e.is_directory())continue;string sn=e.path().filename().string();
        for(auto& f:fs::directory_iterator(e.path())){if(f.path().extension()==".jpg"){string s=f.path().stem().string();if(s.rfind("calib_cam_",0)==0)r.insert(sn+"/"+s);}}}}
    return r;}
int countImages(const string& dir){int n=0;if(!fs::exists(dir))return 0;for(auto& e:fs::directory_iterator(dir))if(e.path().extension()==".jpg")n++;return n;}
// 目录内索引最大的图片完整路径 (内参模式撤回用)
string lastImageIn(const string& dir){string best;int mx=-1;if(!fs::exists(dir))return best;for(auto& e:fs::directory_iterator(dir)){if(e.path().extension()!=".jpg")continue;string s=e.path().stem().string();size_t u=s.find_last_of('_');if(u==string::npos)continue;try{int v=stoi(s.substr(u+1));if(v>mx){mx=v;best=e.path().string();}}catch(...){}}return best;}
// 当前标定模式的图片根目录
string picRoot(){return g_intr_mode.load()?g_intr_root:g_save_dir;}

// ================== UI ==================
void updateLayout(){g_left_w=g_win_h*2/5;g_right_x=g_left_w;g_right_w=g_win_w-g_left_w;g_thumb_w=g_left_w/2;g_thumb_h=g_win_h/5;}
void onMouse(int ev,int x,int y,int,void*){if(ev!=cv::EVENT_LBUTTONDOWN||x>=g_left_w)return;int c=x/g_thumb_w,r=y/g_thumb_h,i=r*2+c;int n=(int)cam_ctxs.size();if(i>=0&&i<n){int p=g_enlarged_cam.load();g_enlarged_cam.store((p==i)?-1:i);}}
void renderGrid(cv::Mat& cv,int sel){int n=(int)cam_ctxs.size();for(int i=0;i<10;++i){int r=i/2,c=i%2,x=c*g_thumb_w,y=r*g_thumb_h;cv::Rect ro(x,y,g_thumb_w,g_thumb_h);
    if(i<n){cv::Mat lr;{lock_guard<mutex> lk(cam_ctxs[i]->frame_mtx);lr=cam_ctxs[i]->latest_frame;}cv::Mat ce;if(!lr.empty()){if(cam_ctxs[i]->is_mono)cv::cvtColor(lr,ce,cv::COLOR_GRAY2RGB);else cv::cvtColor(lr,ce,cv::COLOR_BayerRG2RGB);
        double sc=min((double)g_thumb_w/ce.cols,(double)g_thumb_h/ce.rows);int dw=(int)(ce.cols*sc),dh=(int)(ce.rows*sc);cv::Mat rs;cv::resize(ce,rs,cv::Size(dw,dh));ce=cv::Mat::zeros(g_thumb_h,g_thumb_w,CV_8UC3);int ox=(g_thumb_w-dw)/2,oy=(g_thumb_h-dh)/2;rs.copyTo(ce(cv::Rect(ox,oy,dw,dh)));}
        else{ce=cv::Mat::zeros(g_thumb_h,g_thumb_w,CV_8UC3);int bl=0;cv::Size ts=cv::getTextSize(cam_ctxs[i]->status_msg,cv::FONT_HERSHEY_SIMPLEX,0.5,1,&bl);cv::putText(ce,cam_ctxs[i]->status_msg,cv::Point((g_thumb_w-ts.width)/2,(g_thumb_h+ts.height)/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255),1);}
        string lb=cam_ctxs[i]->sn;int bl=0;cv::Size ts=cv::getTextSize(lb,cv::FONT_HERSHEY_SIMPLEX,0.45,1,&bl);cv::putText(ce,lb,cv::Point((g_thumb_w-ts.width)/2,g_thumb_h-6),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,0),3);cv::putText(ce,lb,cv::Point((g_thumb_w-ts.width)/2,g_thumb_h-6),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(255,255,255),1);
        ce.copyTo(cv(ro));if(i==sel)cv::rectangle(cv,ro,cv::Scalar(0,255,0),2);}else{cv(ro)=cv::Scalar(0,0,0);}}}
void renderEnlarged(cv::Mat& cv,int ci){cv::Rect rr(g_right_x,0,g_right_w,g_win_h);if(ci<0||ci>=(int)cam_ctxs.size()){cv(rr)=cv::Scalar(0,0,0);return;}cv::Mat lr;{lock_guard<mutex> lk(cam_ctxs[ci]->frame_mtx);lr=cam_ctxs[ci]->latest_frame;}if(lr.empty()){cv(rr)=cv::Scalar(0,0,0);int bl=0;cv::Size ts=cv::getTextSize(cam_ctxs[ci]->status_msg,cv::FONT_HERSHEY_DUPLEX,0.9,2,&bl);cv::putText(cv,cam_ctxs[ci]->status_msg,cv::Point(g_right_x+(g_right_w-ts.width)/2,(g_win_h+ts.height)/2),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,215,255),2,cv::LINE_AA);return;}
    cv::Mat im;if(cam_ctxs[ci]->is_mono)cv::cvtColor(lr,im,cv::COLOR_GRAY2RGB);else cv::cvtColor(lr,im,cv::COLOR_BayerRG2RGB);
    double sc=min((double)g_right_w/im.cols,(double)g_win_h/im.rows);int dw=(int)(im.cols*sc),dh=(int)(im.rows*sc);cv::Mat rs;cv::resize(im,rs,cv::Size(dw,dh));int ox=g_right_x+(g_right_w-dw)/2,oy=(g_win_h-dh)/2;rs.copyTo(cv(cv::Rect(ox,oy,dw,dh)));}

// 全屏状态界面: 传输/标定时替代缩略图+放大 UI (不渲染采集画面, 降低 CPU)
// progress: 0~1 画进度条, <0 不画; poll: 同步传输中需要 waitKey 才能刷新窗口
void renderStatusScreen(const string& title,const string& sub1,const string& sub2,double progress,bool poll){
    cv::Mat cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
    int y=g_win_h/2-70,bl=0;
    cv::Size ts=cv::getTextSize(title,cv::FONT_HERSHEY_DUPLEX,1.1,2,&bl);
    cv::putText(cv,title,cv::Point((g_win_w-ts.width)/2,y),cv::FONT_HERSHEY_DUPLEX,1.1,cv::Scalar(0,215,255),2,cv::LINE_AA);y+=50;
    if(!sub1.empty()){ts=cv::getTextSize(sub1,cv::FONT_HERSHEY_SIMPLEX,0.7,1,&bl);cv::putText(cv,sub1,cv::Point((g_win_w-ts.width)/2,y),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(220,220,220),1,cv::LINE_AA);y+=32;}
    if(!sub2.empty()){ts=cv::getTextSize(sub2,cv::FONT_HERSHEY_SIMPLEX,0.7,1,&bl);cv::putText(cv,sub2,cv::Point((g_win_w-ts.width)/2,y),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(220,220,220),1,cv::LINE_AA);y+=32;}
    if(progress>=0.0){y+=10;int bx=500,by=30,x0=(g_win_w-bx)/2;cv::rectangle(cv,cv::Rect(x0,y,bx,by),cv::Scalar(80,80,80),1);int pw=(int)(bx*min(1.0,progress));if(pw>0)cv::rectangle(cv,cv::Rect(x0,y,pw,by),cv::Scalar(0,255,0),-1);char pct[32];snprintf(pct,sizeof(pct),"%.0f%%",progress*100);cv::putText(cv,pct,cv::Point(x0+bx/2-25,y+21),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,255),1);}
    cv::imshow("Calib Capture",cv);
    if(poll)cv::waitKey(1);
}

// 内参模式: 保存放大的相机到 pictures/{SN}/ (返回 false = 无选中/无帧)
bool saveIntrinsicsFrame(){
    int ci=g_enlarged_cam.load();
    if(ci<0||ci>=(int)cam_ctxs.size()){cout<<"[Intrinsics] No camera selected."<<endl;return false;}
    auto& ctx=cam_ctxs[ci];string dir=g_intr_root+"/"+ctx->sn;fs::create_directories(dir);
    int ctr=getNextCounter(dir);stringstream ss;ss<<setw(2)<<setfill('0')<<ctr;
    cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}
    if(snap.empty()){cout<<"[Intrinsics] No frame yet."<<endl;return false;}
    cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);
    string fn=dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg";cv::imwrite(fn,out);
    cout<<"\n[Intrinsics] Saved "<<fn<<endl;
    return true;
}

// ================== TCP: Slave command handler (runs in dedicated thread) ==================
void slaveCmdWorker(SOCKET s){
    while(global_running){
        string line; if(!recvLine(s,line,500)) continue;
        if(line.rfind("PHOTO:",0)==0){int idx=stoi(line.substr(6));stringstream ss;ss<<setw(2)<<setfill('0')<<idx;g_capture_count.store(idx+1);   // 覆盖式: 对齐 master 计数 (文件名必须与 master 一致)
            cout<<"[Slave] PHOTO "<<idx<<endl;
            for(auto& ctx:cam_ctxs){cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}if(!snap.empty()){cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);string fn=g_save_dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg";cv::imwrite(fn,out);cout<<"  -> Saved "<<fn<<endl;}}}
        else if(line.rfind("UNDO:",0)==0){int idx=stoi(line.substr(5));
            cout<<"[Slave] UNDO index "<<idx<<" (counter rollback, files kept for overwrite)"<<endl;
            g_capture_count.store(idx);   // 覆盖式: 对齐 master 回退后的计数
            }
        else if(line.rfind("MODE:",0)==0){string m=line.substr(5);g_intr_mode.store(m=="INTRINSIC");cout<<"[Slave] Calibration mode: "<<(g_intr_mode.load()?"INTRINSIC":"EXTRINSIC")<<endl;}
        else if(line=="IPHOTO"){cout<<"[Slave] IPHOTO request from master"<<endl;
            if(g_intr_mode.load()&&saveIntrinsicsFrame())sendLine(s,"IPHOTO_OK");
            else sendLine(s,"IPHOTO_NOSEL");}
        else if(line=="IUNDO"){cout<<"[Slave] IUNDO request from master"<<endl;
            // 撤回 slave 选中相机的最后一张 (无选中/无图 → NOSEL)
            if(!g_intr_mode.load()){sendLine(s,"IUNDO_NOSEL");continue;}
            int ci=g_enlarged_cam.load();
            if(ci<0||ci>=(int)cam_ctxs.size()){sendLine(s,"IUNDO_NOSEL");continue;}
            string del=lastImageIn(g_intr_root+"/"+cam_ctxs[ci]->sn);
            if(del.empty()){sendLine(s,"IUNDO_NOSEL");continue;}
            fs::remove(del);cout<<"  -> Removed "<<del<<endl;sendLine(s,"IUNDO_OK");}
        else if(line=="ICLEAR"){cout<<"[Slave] ICLEAR — removing ALL intrinsics photos"<<endl;
            // 清空 slave 全部相机的内参图片 (master c 键触发, 双端全清)
            if(fs::exists(g_intr_root))for(auto& d:fs::directory_iterator(g_intr_root)){
                if(!d.is_directory())continue;
                for(auto& e:fs::directory_iterator(d.path()))if(e.path().extension()==".jpg")fs::remove(e.path());}
            sendLine(s,"ICLEAR_DONE");cout<<"[Slave] All intrinsics photos cleared."<<endl;}
        else if(line=="CLEAR"){cout<<"[Slave] CLEAR \xe2\x80\x94 removing all calibration photos"<<endl;if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir))if(e.path().extension()==".jpg")fs::remove(e.path());g_capture_count=0;sendLine(s,"CLEAR_DONE");cout<<"[Slave] Cleared."<<endl;}
        else if(line.rfind("XFER:",0)==0){string lst=line.substr(5);stringstream ss(lst);string tok;vector<string> files;while(getline(ss,tok,','))if(!tok.empty())files.push_back(tok);
            int total=(int)files.size(),cnt=0;size_t total_bytes=0;int data_port=g_ctrl_port+1;
            cout<<"[Slave] Transfer started — "<<total<<" files via data port "<<data_port<<endl;
            SOCKET ds=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
            sockaddr_in dsa{};dsa.sin_family=AF_INET;dsa.sin_port=htons(data_port);inet_pton(AF_INET,g_master_ip.c_str(),&dsa.sin_addr);
            {int rtry=0;while(connect(ds,(sockaddr*)&dsa,sizeof(dsa))!=0){if(++rtry>30){cerr<<"[Slave] Data port connect timeout"<<endl;closesocket(ds);goto xfer_done_ctrl;}this_thread::sleep_for(chrono::milliseconds(200));}}
            for(auto& f:files){string path=picRoot()+"/"+f+".jpg";   // 键含子目录 (内参模式), 保持文件夹结构
                ifstream in(path,ios::binary|ios::ate);if(in){size_t sz=in.tellg();in.seekg(0);vector<char> data(sz);in.read(data.data(),sz);in.close();
                    uint32_t sz_be=htonl((uint32_t)sz);
                    if(!sendExact(ds,&sz_be,4)||!sendExact(ds,data.data(),sz)){cerr<<"[Slave] sendExact FAILED for "<<f<<endl;break;}
                    total_bytes+=sz;cnt++;}
                else{cerr<<"[Slave] Cannot read "<<path<<endl;}}
            {uint32_t zero=0;sendExact(ds,&zero,4);}closesocket(ds);
            xfer_done_ctrl:sendLine(s,"XFER_DONE");cout<<"[Slave] Transfer ended — "<<cnt<<" files, "<<(total_bytes/1048576.0)<<" MB"<<endl;}
        else if(line.rfind("FAULT:",0)==0&&!g_fault_active.load()){/*handle fault*/}
        else if(line=="SHUTDOWN"){cout<<"[Slave] Received SHUTDOWN from master."<<endl;global_running=false;}
    }
}

// ================== main ==================
int main(){
    cout<<"=== Calibration Image Capture (TCP) ==="<<endl;
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2),&wsa);
#endif
    auto cfg_dir=(fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg").string();
    Cfg cfg(cfg_dir+"/cam_calib.yaml"); auto& c=cfg["calib"];
    g_is_master=c["is_master"].as<bool>(); g_master_ip=c["master_ip"].as<string>();
    string sip=c["slave_ip"].as<string>();
    g_ctrl_port=c["port"].as<int>(); g_enable_net_sync=c["enable_net_sync"].as<bool>();
    vector<string> sns=c["cam_indices"].as<vector<string>>();
    string base_dir=c["calib_save_dir"].as<string>();
    string did=c["day_id"].as<string>();   // cam_calib.yaml 自带 (D001 系列, 每天标定一次)
    g_save_dir=base_dir+"/"+did+"/pictures"; fs::create_directories(g_save_dir);
    string participant; try{participant=c["participant_id"].as<string>();}catch(...){participant="P001";}
    g_intr_root=base_dir+"/"+participant+"/pictures";   // 内参模式图片根目录 (per-SN 子目录)
    g_intr_xml_dir=base_dir+"/"+participant+"/output";  // 内参标定 XML 输出目录
    // 标定 XML 输出目录 (cam_calib_chain 的输出, 与 calib_cam_chain.cpp 的计算一致)
    {
        string xdid=did; try{xdid=c["input_day_id"].as<string>();if(xdid.empty())xdid=did;}catch(...){}
        g_xml_dir=base_dir+"/"+xdid+"/output";
    }
    bool test_xfer=false; try{test_xfer=c["test_transfer"].as<bool>();}catch(...){}
    string test_recv; try{test_recv=c["test_transfer_recv_dir"].as<string>();test_recv+="/"+did+"/pictures";}catch(...){test_recv="D:/calib_transfer_test/"+did+"/pictures";}
    if(test_xfer) fs::create_directories(test_recv);
    double fps=c["fps"].as<double>(),gain=c["gain"].as<double>(),gamma=c["gamma"].as<double>(),exp=c["exposure_time"].as<double>(),me=c["calib_mono_exp_ext"].as<double>();
    int data_port=g_ctrl_port+1;
    g_win_w=c["window_width"].as<int>(); g_win_h=c["window_height"].as<int>(); double uif=c["ui_fps"].as<double>();
    cout<<"\n--- Calibration Transfer Configuration ---"<<endl;
    cout<<"Role      : "<<(g_is_master?"MASTER":"SLAVE")<<endl;
    cout<<"Net Sync  : "<<(g_enable_net_sync?"ON":"OFF")<<endl;
    if(g_enable_net_sync){cout<<"Master IP : "<<g_master_ip<<endl;cout<<"Slave IP  : "<<sip<<endl;
        cout<<"Ctrl Port : "<<g_ctrl_port<<endl;cout<<"Data Port : "<<data_port<<endl;}
    cout<<"Test Xfer : "<<(test_xfer?"ON":"OFF")<<endl;
    cout<<"Save dir  : "<<g_save_dir<<endl;
    cout<<"XML dir   : "<<g_xml_dir<<endl;
    cout<<"Intrinsic : "<<participant<<" -> "<<g_intr_root<<endl;
    cout<<"Intr XML  : "<<g_intr_xml_dir<<endl;
    if(test_xfer) cout<<"Recv dir  : "<<test_recv<<endl;
    cout<<"Cameras   : "<<sns.size()<<endl;
    for(size_t i=0;i<sns.size();++i)cout<<"  "<<i<<": SN="<<sns[i]<<endl;
    cout<<"FPS       : "<<fps<<endl;
    cout<<"Exposure  : "<<exp<<" us"<<endl;
    cout<<"Window    : "<<g_win_w<<"x"<<g_win_h<<endl;
    cout<<"------------------------------------------\n"<<endl;

    // ---- TCP handshake ----
    thread cmd_thread;
    if(g_enable_net_sync){
        if(g_is_master){ g_listen_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); int opt=1;setsockopt(g_listen_sock,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
            sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(g_ctrl_port);sa.sin_addr.s_addr=INADDR_ANY;::bind(g_listen_sock,(sockaddr*)&sa,sizeof(sa));listen(g_listen_sock,1);
            cout<<"[TCP] Listening on ::"<<g_ctrl_port<<" ..."<<endl; sockaddr_in ca;socklen_t cl=sizeof(ca);g_ctrl_sock=accept(g_listen_sock,(sockaddr*)&ca,&cl);
            string hl; recvLine(g_ctrl_sock,hl,10000); if(hl=="READY") sendLine(g_ctrl_sock,"ACK"); else{cerr<<"[Error] TCP handshake failed"<<endl;return 1;}
            cout<<"[TCP] Slave connected. Handshake OK."<<endl;}
        else{ g_ctrl_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(g_ctrl_port);inet_pton(AF_INET,g_master_ip.c_str(),&sa.sin_addr);
            cout<<"[TCP] Connecting to "<<g_master_ip<<":"<<g_ctrl_port<<" ..."<<endl;
            int retry=0; while(connect(g_ctrl_sock,(sockaddr*)&sa,sizeof(sa))!=0){if(++retry%10==1)cerr<<"[TCP] Connect retry #"<<retry<<"..."<<endl;this_thread::sleep_for(chrono::milliseconds(500));}
            sendLine(g_ctrl_sock,"READY"); string hl; recvLine(g_ctrl_sock,hl,10000); if(hl!="ACK"){cerr<<"[Error] TCP handshake failed"<<endl;return 1;}
            cout<<"[TCP] Connected to Master. Handshake OK."<<endl;cmd_thread=thread(slaveCmdWorker,g_ctrl_sock);}
    }

    // ---- test_transfer mode (skip cameras, show transfer UI) ----
    if(test_xfer){
        cout<<"[System] Test transfer mode — skipping camera init."<<endl;
        bool xfer_done=false; string xfer_status; int xfer_cnt=0; size_t xfer_bytes=0; int xfer_total=0; double xfer_speed=0;
        cv::namedWindow("Calib Transfer Test",cv::WINDOW_NORMAL);cv::resizeWindow("Calib Transfer Test",800,400);
        auto uii=chrono::milliseconds((int)(1000.0/uif));auto lui=chrono::steady_clock::now()-uii;
        while(global_running){
            auto now=chrono::steady_clock::now();bool nu=(now-lui)>=uii;
            if(nu){cv::Mat cv=cv::Mat::zeros(400,800,CV_8UC3);int y=60;
                cv::putText(cv,"TCP Transfer Speed Test",cv::Point(200,y),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,255,255),2);y+=40;
                if(!xfer_done){cv::putText(cv,"Press [T] to start transfer  [Q] to quit",cv::Point(150,y),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(200,200,200),1);y+=30;
                    if(!xfer_status.empty()){cv::putText(cv,xfer_status,cv::Point(100,y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);y+=24;}}
                else{char b[128];snprintf(b,sizeof(b),"DONE: %d files, %.1f MB in %.1fs (%.1f MB/s)",xfer_cnt,xfer_bytes/1048576.0,(xfer_bytes/1048576.0)/(xfer_speed>0?xfer_speed:1),xfer_speed);cv::putText(cv,b,cv::Point(80,y),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,0),2);y+=40;
                    cv::putText(cv,"Press [T] to test again  [Q] to quit",cv::Point(180,y),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(200,200,200),1);}
                // Progress bar
                if(xfer_total>0){cv::rectangle(cv,cv::Rect(100,300,600,30),cv::Scalar(80,80,80),1);int pw=(int)(600.0*xfer_cnt/xfer_total);cv::rectangle(cv,cv::Rect(100,300,pw,30),cv::Scalar(0,255,0),-1);char pct[32];snprintf(pct,sizeof(pct),"%d/%d",xfer_cnt,xfer_total);cv::putText(cv,pct,cv::Point(340,320),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,255),1);}
                // Bottom-right: save dir + XML dir
                cv::putText(cv,"Save: "+g_save_dir,cv::Point(20,385),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(140,140,140),1,cv::LINE_AA);
                cv::putText(cv,"XML : "+g_xml_dir,cv::Point(20,398),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(140,140,140),1,cv::LINE_AA);
                cv::imshow("Calib Transfer Test",cv);lui=now;}
            char key=(char)cv::waitKey(30);
            if(key=='q'||key==27){if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"SHUTDOWN");global_running=false;}
            else if((key=='c'||key=='C')&&g_is_master){xfer_cnt=0;xfer_bytes=0;xfer_total=0;xfer_done=false;xfer_status="";
                cout<<"\n[Clear] Removing test transfer images..."<<endl;
                if(fs::exists(test_recv))for(auto& e:fs::directory_iterator(test_recv))if(e.path().extension()==".jpg")fs::remove(e.path());
                cout<<"[Clear] Local photos removed.\n"<<endl;}
            else if((key=='t'||key=='T')&&g_is_master&&g_enable_net_sync){
                xfer_status="Transferring...";xfer_cnt=0;xfer_bytes=0;xfer_done=false;
                sendLine(g_ctrl_sock,"LIST_REQ");
                string resp; if(!recvLine(g_ctrl_sock,resp,5000)){cerr<<"[Error] No response from slave."<<endl;continue;}
                if(resp.rfind("LIST_RESP:",0)!=0){xfer_status="LIST_REQ failed";continue;}
                string lst=resp.substr(10);set<string> sf;stringstream ss(lst);string tok;while(getline(ss,tok,','))if(!tok.empty())sf.insert(tok);
                set<string> lf=scanFiles(test_recv,false);vector<string> mis;for(auto& f:sf)if(!lf.count(f))mis.push_back(f);
                if(mis.empty()){xfer_status="Already in sync.";xfer_done=true;continue;}
                xfer_total=(int)mis.size();xfer_status="Transferring "+to_string(xfer_total)+" files...";
                cout<<"\n[Transfer] "<<mis.size()<<" files missing. Starting transfer..."<<endl;
                string xfer="XFER:";for(auto& f:mis)xfer+=f+",";sendLine(g_ctrl_sock,xfer);
                // Listen on data port for binary file stream
                SOCKET dl=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
                {int opt=1;setsockopt(dl,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
                sockaddr_in dsa{};dsa.sin_family=AF_INET;dsa.sin_port=htons(data_port);dsa.sin_addr.s_addr=INADDR_ANY;
                ::bind(dl,(sockaddr*)&dsa,sizeof(dsa));listen(dl,1);}
                SOCKET ds=accept(dl,nullptr,nullptr);
                auto t0=chrono::steady_clock::now();int file_idx=0;
                while(true){uint32_t sz_be;if(!recvExact(ds,&sz_be,4)){cerr<<"[XFER] Failed to read size header"<<endl;break;}
                    size_t sz=ntohl(sz_be);if(sz==0)break;
                    if(file_idx>=(int)mis.size()){cerr<<"[XFER] Too many files!"<<endl;break;}
                    vector<char> buf(sz);if(!recvExact(ds,buf.data(),sz)){cerr<<"[XFER] recvExact FAILED at file "<<file_idx<<endl;break;}
                    xfer_bytes+=sz;xfer_cnt++;file_idx++;
                    auto& f=mis[file_idx-1];
                    string path=test_recv+"/"+f+".jpg";ofstream out(path,ios::binary);out.write(buf.data(),sz);
                    auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();xfer_speed=dt>0?(xfer_bytes/1048576.0)/dt:0;
                    if(xfer_cnt%50==0||xfer_cnt==xfer_total)cout<<"[XFER] Progress: "<<xfer_cnt<<"/"<<xfer_total<<" ("<<(xfer_bytes/1048576.0)<<" MB, "<<xfer_speed<<" MB/s)"<<endl;}
                closesocket(ds);closesocket(dl);
                {string l;recvLine(g_ctrl_sock,l,10000);} // consume XFER_DONE
                auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();xfer_speed=dt>0?(xfer_bytes/1048576.0)/dt:0;xfer_done=true;
                cout<<"[Transfer] "<<xfer_cnt<<" files, "<<fixed<<setprecision(1)<<(xfer_bytes/1048576.0)<<" MB, "<<dt<<"s, "<<xfer_speed<<" MB/s\n"<<endl;
            }
        }
        if(cmd_thread.joinable())cmd_thread.join();
        if(g_listen_sock!=INVALID_SOCKET)closesocket(g_listen_sock);if(g_ctrl_sock!=INVALID_SOCKET)closesocket(g_ctrl_sock);
        cv::destroyAllWindows();WSACleanup();return 0;
    }

    // ---- Camera init (non-test mode) ----
    cout<<"[System] Initializing cameras..."<<endl;
    Pylon::PylonInitialize();
    for(size_t i=0;i<sns.size();++i){auto ctx=make_shared<CameraContext>(sns[i]);cam_ctxs.push_back(ctx);}
    for(auto& ctx:cam_ctxs){ctx->running=true;ctx->copy_thread=thread(copyWorker,ctx);ctx->capture_thread=thread(captureWorker,ctx,fps,gain,gamma,exp,me);}

    // ---- UI ----
    cv::namedWindow("Calib Capture",cv::WINDOW_NORMAL);cv::resizeWindow("Calib Capture",g_win_w,g_win_h);updateLayout();cv::setMouseCallback("Calib Capture",onMouse);
    auto uii=chrono::milliseconds((int)(1000.0/uif));auto lui=chrono::steady_clock::now()-uii;
    g_capture_count=getNextCounter(g_save_dir);   // 启动时: 计数 = 最大后缀 + 1 = 下一张后缀
    cout<<"[System] Existing captures: "<<g_capture_count<<endl;g_ready_time=chrono::steady_clock::now();

    while(global_running){
        auto now=chrono::steady_clock::now();bool nu=(now-lui)>=uii;
        // UI
        if(nu){
            if(g_ui_mode==UiMode::CALIBRATING){char et[64];snprintf(et,sizeof(et),"Elapsed: %.1fs",chrono::duration<double>(now-g_calib_start).count());renderStatusScreen("CALIBRATING...",g_calib_title,et,-1,false);}
            else{cv::Mat cv;if(g_fault_active.load()){cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);cv::putText(cv,"CAMERA FAULT",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,1.2,cv::Scalar(0,0,255),2);}
            else{cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);int sel=g_enlarged_cam.load();renderGrid(cv,sel);renderEnlarged(cv,sel);cv::line(cv,cv::Point(g_left_w,0),cv::Point(g_left_w,g_win_h),cv::Scalar(60,60,60),2);
                // 中心十字线 (enlarged 区域中心, 与 capture_with_LED 一致)
                int cx=g_right_x+g_right_w/2,cy=g_win_h/2,cl=20;
                cv::line(cv,cv::Point(cx-cl,cy),cv::Point(cx+cl,cy),cv::Scalar(100,100,100),1,cv::LINE_AA);
                cv::line(cv,cv::Point(cx,cy-cl),cv::Point(cx,cy+cl),cv::Scalar(100,100,100),1,cv::LINE_AA);
                // 左上: 相机 SN (如有放大) + 标定模式 (始终)
                if(sel>=0&&sel<(int)cam_ctxs.size())cv::putText(cv,cam_ctxs[sel]->sn,cv::Point(g_right_x+10,35),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,215,255),2,cv::LINE_AA);
                string mode_text=g_intr_mode.load()?"MODE: INTRINSIC":"MODE: EXTRINSIC";
                cv::Scalar mode_color=g_intr_mode.load()?cv::Scalar(0,255,0):cv::Scalar(200,80,255);
                cv::putText(cv,mode_text,cv::Point(g_right_x+10,62),cv::FONT_HERSHEY_SIMPLEX,0.5,mode_color,2,cv::LINE_AA);
                // 右上: 图片数量 (内参模式 = 放大相机; 外参模式 = 全部)
                string cnt=g_intr_mode.load()?(sel>=0&&sel<(int)cam_ctxs.size()?"Pictures: "+to_string(countImages(g_intr_root+"/"+cam_ctxs[sel]->sn)):"Pictures: -"):"Captures: "+to_string(g_capture_count);
                cv::Size csz=cv::getTextSize(cnt,cv::FONT_HERSHEY_SIMPLEX,0.8,2,0);
                cv::putText(cv,cnt,cv::Point(g_right_x+g_right_w-csz.width-10,35),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,215,255),2,cv::LINE_AA);
                string hints;
                if(g_intr_mode.load())hints=g_enable_net_sync&&!g_is_master?"[click] select camera (master SPACE to save)  [ESC/q] quit":"[SPACE] save enlarged (none sel. -> slave)  [z] undo  [c] clear  [t] transfer  [v] viz  [i] extrinsics  [ESC/q] quit";
                else hints=g_enable_net_sync&&!g_is_master?"[SPACE][z][c][t][ESC/q] disabled (Slave)":"[SPACE] capture  [z] undo  [c] clear  [t] transfer+calib  [v] viz  [i] intrinsics  [ESC/q] quit";
                cv::putText(cv,hints,cv::Point(g_right_x+10,g_win_h-45),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(140,140,140),1,cv::LINE_AA);
                // Bottom-right: save dir + XML dir (右对齐; 内参模式显示 participant 目录)
                string sdir=g_intr_mode.load()?"Save: "+g_intr_root:"Save: "+g_save_dir;cv::Size ssz=cv::getTextSize(sdir,cv::FONT_HERSHEY_SIMPLEX,0.35,1,0);
                cv::putText(cv,sdir,cv::Point(g_right_x+g_right_w-ssz.width-10,g_win_h-35),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(140,140,140),1,cv::LINE_AA);
                string xdir=g_intr_mode.load()?"XML : "+g_intr_xml_dir:"XML : "+g_xml_dir;cv::Size xsz=cv::getTextSize(xdir,cv::FONT_HERSHEY_SIMPLEX,0.35,1,0);
                cv::putText(cv,xdir,cv::Point(g_right_x+g_right_w-xsz.width-10,g_win_h-20),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(140,140,140),1,cv::LINE_AA);}
            cv::imshow("Calib Capture",cv);}
            lui=now;}
#ifdef _WIN32
        // HALCON 子进程退出检测 → 自动回到采集界面
        if(g_ui_mode==UiMode::CALIBRATING){
            PROCESS_INFORMATION* pi=nullptr; string disp;
            if(g_halcon_pi.hProcess){pi=&g_halcon_pi;disp="cam_calib_chain";}
            else if(g_intrinsics_pi.hProcess){pi=&g_intrinsics_pi;disp="calib_cam_intrinsics";}
            if(pi){DWORD ec=0;
                if(GetExitCodeProcess(pi->hProcess,&ec)&&ec!=STILL_ACTIVE){
                    cout<<"[HALCON] "<<disp<<" finished (exit code "<<ec<<")."<<endl;
                    CloseHandle(pi->hProcess);CloseHandle(pi->hThread);ZeroMemory(pi,sizeof(*pi));
                    g_ui_mode=UiMode::CAPTURE;
                }}
        }
#endif

        char key=(char)cv::waitKey(1);
        if(key==27||key=='q'){if(g_enable_net_sync&&g_is_master){sendLine(g_ctrl_sock,"SHUTDOWN");this_thread::sleep_for(chrono::milliseconds(200));}if(!g_enable_net_sync||g_is_master||g_fault_active.load())global_running=false;}
        else if(g_fault_active.load()){}
        else if(g_ui_mode==UiMode::CALIBRATING){}   // 标定期间忽略其他按键
        else if(key=='i'||key=='I'){
            if(g_is_master||!g_enable_net_sync){
                g_intr_mode.store(!g_intr_mode.load());
                if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,g_intr_mode.load()?"MODE:INTRINSIC":"MODE:EXTRINSIC");
                cout<<"\n[Mode] "<<(g_intr_mode.load()?"INTRINSIC":"EXTRINSIC")<<" calibration mode"<<endl;
            }
        }
        else if(key=='v'||key=='V'){
            if(g_is_master||!g_enable_net_sync){
                // 外参模式可视化 day 结果, 内参模式可视化 participant 结果
                string viz_id=g_intr_mode.load()?fs::path(g_intr_root).parent_path().filename().string():fs::path(g_xml_dir).parent_path().filename().string();
                launchViz(viz_id);
            }
        }
        else if(key==' '){
            if(g_intr_mode.load()){
                if(g_enable_net_sync&&!g_is_master){/* slave 内参模式: 按键禁用, 只负责选中相机 */}
                else if(g_enlarged_cam.load()<0&&g_enable_net_sync&&g_is_master){
                    // master 未选中相机 → 命令 slave 对其选中相机拍照保存
                    sendLine(g_ctrl_sock,"IPHOTO");string resp;recvLine(g_ctrl_sock,resp,3000);
                    if(resp=="IPHOTO_OK")cout<<"\n[Intrinsics] Slave saved its selected camera."<<endl;
                    else if(resp=="IPHOTO_NOSEL")cout<<"[Intrinsics] Slave has no camera selected."<<endl;
                    else cout<<"[Intrinsics] No response from slave."<<endl;
                }
                else{saveIntrinsicsFrame();}   // 保存本机放大相机的画面
            }
            else if(g_enable_net_sync&&!g_is_master){/* 外参模式: slave 由 master PHOTO 命令驱动 */}
            else{int ctr=g_capture_count.load();stringstream ss;ss<<setw(2)<<setfill('0')<<ctr;if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"PHOTO:"+to_string(ctr));g_capture_count++;
            cout<<"\n[Photo] Capturing index "<<ctr<<endl;
            for(auto& ctx:cam_ctxs){cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}if(!snap.empty()){cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);string fn=g_save_dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg";cv::imwrite(fn,out);cout<<"  -> Saved "<<fn<<endl;}}}}
        else if((key=='t'||key=='T')&&g_is_master&&g_enable_net_sync){
            // 传输期间全屏显示进度, 不渲染采集 UI
            renderStatusScreen("TRANSFERRING...","Requesting file list from slave","",-1,true);
            sendLine(g_ctrl_sock,"LIST_REQ");string resp;recvLine(g_ctrl_sock,resp,5000);
            if(resp.rfind("LIST_RESP:",0)!=0){renderStatusScreen("TRANSFER FAILED","No response from slave","",-1,true);this_thread::sleep_for(chrono::milliseconds(800));continue;}
            string lst=resp.substr(10);set<string> sf;stringstream ss(lst);string tok;while(getline(ss,tok,','))if(!tok.empty())sf.insert(tok);set<string> lf=scanFiles(picRoot(),g_intr_mode.load());vector<string> mis;for(auto& f:sf)if(!lf.count(f))mis.push_back(f);
            if(mis.empty()){cout<<"[Transfer] Already in sync."<<endl;
                if(g_intr_mode.load()){renderStatusScreen("IN SYNC","No files missing","Starting intrinsics calibration...",-1,true);if(launchIntrinsicsCalib()){g_calib_title="calib_cam_intrinsics is running";g_ui_mode=UiMode::CALIBRATING;g_calib_start=chrono::steady_clock::now();}}
                else{renderStatusScreen("IN SYNC","No files missing","Starting calibration...",-1,true);if(launchHalconChain()){g_calib_title="cam_calib_chain is running";g_ui_mode=UiMode::CALIBRATING;g_calib_start=chrono::steady_clock::now();}}
                continue;}
            cout<<"\n[Transfer] "<<mis.size()<<" files missing. Starting transfer..."<<endl;string xfer="XFER:";for(auto& f:mis)xfer+=f+",";sendLine(g_ctrl_sock,xfer);
            SOCKET dl=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
            {int opt=1;setsockopt(dl,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
            sockaddr_in dsa{};dsa.sin_family=AF_INET;dsa.sin_port=htons(data_port);dsa.sin_addr.s_addr=INADDR_ANY;
            ::bind(dl,(sockaddr*)&dsa,sizeof(dsa));listen(dl,1);}
            SOCKET ds=accept(dl,nullptr,nullptr);
            int rc=0,fi=0;size_t tb=0;auto t0=chrono::steady_clock::now();auto last_ui=t0;
            while(true){uint32_t sz_be;if(!recvExact(ds,&sz_be,4)){cerr<<"[XFER] recvExact size header failed"<<endl;break;}
                size_t sz=ntohl(sz_be);if(sz==0)break;
                if(fi>=(int)mis.size()){cerr<<"[XFER] Too many files!"<<endl;break;}
                vector<char> buf(sz);if(!recvExact(ds,buf.data(),sz)){cerr<<"[XFER] recvExact data failed at file "<<fi<<endl;break;}
                tb+=sz;rc++;fi++;auto& f=mis[fi-1];
                string path=picRoot()+"/"+f+".jpg";fs::create_directories(fs::path(path).parent_path());ofstream out(path,ios::binary);out.write(buf.data(),sz);   // 键含子目录, 保持文件夹结构
                // 每 25 张或 200ms 刷新一次全屏传输进度
                auto now2=chrono::steady_clock::now();
                if(rc%25==0||now2-last_ui>chrono::milliseconds(200)){char p[96];snprintf(p,sizeof(p),"%d / %d files, %.1f MB",rc,(int)mis.size(),tb/1048576.0);renderStatusScreen("TRANSFERRING...",p,"",rc/(double)mis.size(),true);last_ui=now2;}}
            closesocket(ds);closesocket(dl);
            {string l;recvLine(g_ctrl_sock,l,10000);} // consume XFER_DONE
            auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();double spd=dt>0?(tb/1048576.0/dt):0;
            cout<<"[Transfer] "<<rc<<" files, "<<fixed<<setprecision(1)<<(tb/1048576.0)<<" MB, "<<dt<<"s, "<<spd<<" MB/s\n"<<endl;
            char p[128];snprintf(p,sizeof(p),"Done: %d files, %.1f MB, %.1fs, %.1f MB/s",rc,tb/1048576.0,dt,spd);
            if(g_intr_mode.load()){
                // 传输完成 → 启动内参重标定 (calib_cam_intrinsics.exe)
                renderStatusScreen("TRANSFER COMPLETE",p,"Starting intrinsics calibration...",-1,true);
                if(launchIntrinsicsCalib()){g_calib_title="calib_cam_intrinsics is running";g_ui_mode=UiMode::CALIBRATING;g_calib_start=chrono::steady_clock::now();}}
            else{
                // 传输完成 → 启动 HALCON 标定链 (标定期间全屏 CALIBRATING, 不渲染采集 UI)
                renderStatusScreen("TRANSFER COMPLETE",p,"Starting calibration...",-1,true);
                if(launchHalconChain()){g_calib_title="cam_calib_chain is running";g_ui_mode=UiMode::CALIBRATING;g_calib_start=chrono::steady_clock::now();}}}
        else if((key=='z'||key=='Z')&&!g_fault_active.load()){
            if(g_intr_mode.load()){
                if(g_enable_net_sync&&!g_is_master){}   // slave 按键禁用
                else if(g_enlarged_cam.load()<0&&g_enable_net_sync&&g_is_master){
                    // master 未选中 → 命令 slave 撤回其选中相机的最后一张
                    sendLine(g_ctrl_sock,"IUNDO");string resp;recvLine(g_ctrl_sock,resp,3000);
                    if(resp=="IUNDO_OK")cout<<"\n[Intrinsics] Slave undid its selected camera."<<endl;
                    else if(resp=="IUNDO_NOSEL")cout<<"[Intrinsics] Slave: no camera selected / no images."<<endl;
                    else cout<<"[Intrinsics] No response from slave."<<endl;
                }
                else{int ci=g_enlarged_cam.load();
                if(ci<0||ci>=(int)cam_ctxs.size()){cout<<"[Intrinsics] No camera selected."<<endl;continue;}
                string dir=g_intr_root+"/"+cam_ctxs[ci]->sn;string del=lastImageIn(dir);
                if(del.empty()){cout<<"[Intrinsics] No images to undo."<<endl;continue;}
                fs::remove(del);
                cout<<"\n[Intrinsics] Undo: removed "<<del<<endl;}
            }
            else{int li=g_capture_count.load()-1;if(li<0){cout<<"[Undo] No previous capture to undo."<<endl;continue;}
            cout<<"\n[Undo] Rolling back index "<<li<<" (next capture will overwrite)"<<endl;
            g_capture_count--;g_undo_count++;
            if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"UNDO:"+to_string(li));   // slave 同步计数器回退
            cout<<"[Undo] Done.\n"<<endl;}}
        else if(key=='c'||key=='C'){
            if(g_intr_mode.load()){
                if(g_enable_net_sync&&!g_is_master){}   // slave 按键禁用
                else if(g_enlarged_cam.load()<0&&g_enable_net_sync&&g_is_master){
                    // master 未选中 → 清空 master + slave 全部相机的内参照片
                    cout<<"\n[Intrinsics] Clearing ALL cameras on master and slave..."<<endl;
                    if(fs::exists(g_intr_root))for(auto& d:fs::directory_iterator(g_intr_root)){
                        if(!d.is_directory())continue;
                        for(auto& e:fs::directory_iterator(d.path()))if(e.path().extension()==".jpg")fs::remove(e.path());}
                    sendLine(g_ctrl_sock,"ICLEAR");string resp;recvLine(g_ctrl_sock,resp,5000);
                    cout<<(resp=="ICLEAR_DONE"?"[Intrinsics] Slave cleared.":"[Intrinsics] Slave no response ("+resp+").")<<endl;
                    cout<<"[Intrinsics] All intrinsics photos cleared."<<endl;
                }
                else{int ci=g_enlarged_cam.load();
                if(ci<0||ci>=(int)cam_ctxs.size()){cout<<"[Intrinsics] No camera selected."<<endl;continue;}
                string dir=g_intr_root+"/"+cam_ctxs[ci]->sn;int n=countImages(dir);
                if(fs::exists(dir))for(auto& e:fs::directory_iterator(dir))if(e.path().extension()==".jpg")fs::remove(e.path());
                cout<<"\n[Intrinsics] Cleared "<<n<<" images from "<<dir<<endl;}
            }
            else if(g_is_master&&g_enable_net_sync){
            cout<<"\n[Clear] Removing local calibration photos..."<<endl;
            if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir))if(e.path().extension()==".jpg")fs::remove(e.path());g_capture_count=0;sendLine(g_ctrl_sock,"CLEAR");string ack;recvLine(g_ctrl_sock,ack,2000);
            cout<<"[Clear] Local photos removed.\n"<<endl;}}
    }

    cout<<"[System] Shutting down..."<<endl;
    for(auto& ctx:cam_ctxs){ctx->running=false;ctx->copy_cv.notify_all();if(ctx->capture_thread.joinable())ctx->capture_thread.join();if(ctx->copy_thread.joinable())ctx->copy_thread.join();}
    if(cmd_thread.joinable())cmd_thread.join();
    if(g_listen_sock!=INVALID_SOCKET)closesocket(g_listen_sock);if(g_ctrl_sock!=INVALID_SOCKET)closesocket(g_ctrl_sock);
    cv::destroyAllWindows();Pylon::PylonTerminate();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
