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
#include <system_error>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== Globals ==================
atomic<bool> global_running{true};
bool g_is_master = false, g_enable_net_sync = false;
string g_save_dir;  // calib_save_dir/{participant_id}
string g_xml_dir;   // 标定 XML 输出目录 = cam_calib.yaml: calib_save_dir/{input_participant_id}/output
int g_win_w=1600, g_win_h=800, g_left_w, g_right_x, g_right_w, g_thumb_w, g_thumb_h;
atomic<int> g_enlarged_cam{-1};
SOCKET g_ctrl_sock = INVALID_SOCKET;  // TCP control channel (text commands)
SOCKET g_listen_sock = INVALID_SOCKET;
int g_ctrl_port = 0;                 // control port; data port = g_ctrl_port + 1
string g_master_ip;                  // Slave needs this to connect to data port
atomic<bool> g_fault_active{false}; atomic<int> g_faulty_cam{-1};
atomic<bool> g_fault_on_master{false};
chrono::steady_clock::time_point g_fault_time, g_ready_time;
atomic<int> g_capture_count{-1}, g_last_idx{-1}; int g_undo_count=0;
#ifdef _WIN32
PROCESS_INFORMATION g_halcon_pi{};   // calib_cam_chain 子进程 (t 键传输完成后触发)
#endif

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

// ================== HALCON 标定链子进程 ==================
#ifdef _WIN32
// 启动同目录下的 calib_cam_chain.exe (已运行则跳过)
void launchHalconChain() {
    if (g_halcon_pi.hProcess) {
        DWORD ec = 0;
        if (GetExitCodeProcess(g_halcon_pi.hProcess, &ec) && ec == STILL_ACTIVE) {
            cout << "[HALCON] calib_cam_chain already running. Please wait." << endl;
            return;
        }
        CloseHandle(g_halcon_pi.hProcess); CloseHandle(g_halcon_pi.hThread);
        ZeroMemory(&g_halcon_pi, sizeof(g_halcon_pi));
    }
    char self_path[MAX_PATH];
    GetModuleFileNameA(NULL, self_path, MAX_PATH);
    fs::path exe = fs::path(self_path).parent_path() / "calib_cam_chain.exe";
    if (!fs::exists(exe)) {
        cerr << "[HALCON] Not found: " << exe.string()
             << " (需设置 HALCONROOT 并编译 cam_calib_chain)" << endl;
        return;
    }
    STARTUPINFOA si{}; si.cb = sizeof(si);
    if (CreateProcessA(exe.string().c_str(), NULL, NULL, NULL, FALSE, 0,
                       NULL, NULL, &si, &g_halcon_pi)) {
        cout << "[HALCON] Launched calib_cam_chain.exe" << endl;
    } else {
        cerr << "[HALCON] Launch failed (error " << GetLastError() << "): "
             << exe.string() << endl;
    }
}
#else
void launchHalconChain() { cerr << "[HALCON] Not supported on this platform." << endl; }
#endif

// ================== Camera ==================
struct CameraContext {string sn; BaslerCamera cam{""}; bool is_mono=true; thread capture_thread,copy_thread; atomic<bool> running{true};
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
    if(!ctx->cam.open(TriggerMode::Software))return;ctx->is_mono=ctx->cam.isMono();if(ctx->is_mono&&me>1.0){exp*=me;fps/=me;}
    try{ctx->cam.setFrameRate(fps);ctx->cam.setGain(gain);ctx->cam.setGamma(gamma);ctx->cam.setExposureTime(exp);}catch(...){}
    // 回调内同步拷贝: 采集卡缓冲只在回调期间被持有, 随后立即归还驱动
    ctx->cam.setFrameCallback([ctx](const Pylon::CBaslerUniversalGrabResultPtr& p,FrameMeta m){
        cv::Mat tmp(p->GetHeight(),p->GetWidth(),CV_8UC1,p->GetBuffer());
        cv::Mat clone_img=tmp.clone();
        lock_guard<mutex> lk(ctx->copy_mtx);
        if(ctx->copy_queue.size()<2){ctx->copy_queue.push({clone_img,m});ctx->copy_cv.notify_one();}
    });
    if(!ctx->cam.start())return;while(ctx->running)this_thread::sleep_for(chrono::milliseconds(50));ctx->cam.close();}

int getNextCounter(const string& dir){int mx=-1;if(!fs::exists(dir))return 0;for(auto& e:fs::directory_iterator(dir)){if(e.path().extension()==".jpg"){try{string s=e.path().stem().string();size_t u=s.find_last_of('_');if(u!=string::npos)mx=max(mx,stoi(s.substr(u+1)));}catch(...){}}}return mx+1;}
set<string> scanFiles(const string& dir){set<string> r;if(!fs::exists(dir))return r;for(auto& e:fs::directory_iterator(dir)){if(e.path().extension()==".jpg"){string s=e.path().stem().string();if(s.rfind("calib_cam_",0)==0)r.insert(s.substr(10));}}return r;}

// ================== UI ==================
void updateLayout(){g_left_w=g_win_h*2/5;g_right_x=g_left_w;g_right_w=g_win_w-g_left_w;g_thumb_w=g_left_w/2;g_thumb_h=g_win_h/5;}
void onMouse(int ev,int x,int y,int,void*){if(ev!=cv::EVENT_LBUTTONDOWN||x>=g_left_w)return;int c=x/g_thumb_w,r=y/g_thumb_h,i=r*2+c;int n=(int)cam_ctxs.size();if(i>=0&&i<n){int p=g_enlarged_cam.load();g_enlarged_cam.store((p==i)?-1:i);}}
void renderGrid(cv::Mat& cv,int sel){int n=(int)cam_ctxs.size();for(int i=0;i<10;++i){int r=i/2,c=i%2,x=c*g_thumb_w,y=r*g_thumb_h;cv::Rect ro(x,y,g_thumb_w,g_thumb_h);
    if(i<n){cv::Mat lr;{lock_guard<mutex> lk(cam_ctxs[i]->frame_mtx);lr=cam_ctxs[i]->latest_frame;}cv::Mat ce;if(!lr.empty()){if(cam_ctxs[i]->is_mono)cv::cvtColor(lr,ce,cv::COLOR_GRAY2RGB);else cv::cvtColor(lr,ce,cv::COLOR_BayerRG2RGB);
        double sc=min((double)g_thumb_w/ce.cols,(double)g_thumb_h/ce.rows);int dw=(int)(ce.cols*sc),dh=(int)(ce.rows*sc);cv::Mat rs;cv::resize(ce,rs,cv::Size(dw,dh));ce=cv::Mat::zeros(g_thumb_h,g_thumb_w,CV_8UC3);int ox=(g_thumb_w-dw)/2,oy=(g_thumb_h-dh)/2;rs.copyTo(ce(cv::Rect(ox,oy,dw,dh)));}
        else{ce=cv::Mat::zeros(g_thumb_h,g_thumb_w,CV_8UC3);}
        string lb=cam_ctxs[i]->sn;int bl=0;cv::Size ts=cv::getTextSize(lb,cv::FONT_HERSHEY_SIMPLEX,0.45,1,&bl);cv::putText(ce,lb,cv::Point((g_thumb_w-ts.width)/2,g_thumb_h-6),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,0),3);cv::putText(ce,lb,cv::Point((g_thumb_w-ts.width)/2,g_thumb_h-6),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(255,255,255),1);
        ce.copyTo(cv(ro));if(i==sel)cv::rectangle(cv,ro,cv::Scalar(0,255,0),2);}else{cv(ro)=cv::Scalar(0,0,0);}}}
void renderEnlarged(cv::Mat& cv,int ci){cv::Rect rr(g_right_x,0,g_right_w,g_win_h);if(ci<0||ci>=(int)cam_ctxs.size()){cv(rr)=cv::Scalar(0,0,0);return;}cv::Mat lr;{lock_guard<mutex> lk(cam_ctxs[ci]->frame_mtx);lr=cam_ctxs[ci]->latest_frame;}if(lr.empty()){cv(rr)=cv::Scalar(0,0,0);return;}
    cv::Mat im;if(cam_ctxs[ci]->is_mono)cv::cvtColor(lr,im,cv::COLOR_GRAY2RGB);else cv::cvtColor(lr,im,cv::COLOR_BayerRG2RGB);
    double sc=min((double)g_right_w/im.cols,(double)g_win_h/im.rows);int dw=(int)(im.cols*sc),dh=(int)(im.rows*sc);cv::Mat rs;cv::resize(im,rs,cv::Size(dw,dh));int ox=g_right_x+(g_right_w-dw)/2,oy=(g_win_h-dh)/2;rs.copyTo(cv(cv::Rect(ox,oy,dw,dh)));}

// ================== TCP: Slave command handler (runs in dedicated thread) ==================
void slaveCmdWorker(SOCKET s){
    while(global_running){
        string line; if(!recvLine(s,line,500)) continue;
        if(line.rfind("PHOTO:",0)==0){int idx=stoi(line.substr(6));stringstream ss;ss<<setw(2)<<setfill('0')<<idx;g_last_idx.store(idx);g_capture_count++;
            cout<<"[Slave] PHOTO "<<idx<<endl;
            for(auto& ctx:cam_ctxs){cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}if(!snap.empty()){cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);string fn=g_save_dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg";cv::imwrite(fn,out);cout<<"  -> Saved "<<fn<<endl;}}}
        else if(line.rfind("UNDO:",0)==0){int idx=stoi(line.substr(5));stringstream ss;ss<<setw(2)<<setfill('0')<<idx;
            cout<<"[Slave] UNDO index "<<ss.str()<<endl;
            if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir)){string st=e.path().stem().string();if(st.rfind("calib_cam_",0)==0&&st.length()>=2&&st.substr(st.length()-2)==ss.str())fs::remove(e.path());}g_last_idx.store(idx-1);if(g_capture_count>0)g_capture_count--;}
        else if(line=="LIST_REQ"){cout<<"[Slave] LIST request"<<endl;stringstream fl;for(auto& f:scanFiles(g_save_dir))fl<<f<<",";sendLine(s,"LIST_RESP:"+fl.str());cout<<"[Slave] Sent file list"<<endl;}
        else if(line=="CLEAR"){cout<<"[Slave] CLEAR \xe2\x80\x94 removing all calibration photos"<<endl;if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir))if(e.path().extension()==".jpg")fs::remove(e.path());g_capture_count=0;g_last_idx=-1;sendLine(s,"CLEAR_DONE");cout<<"[Slave] Cleared."<<endl;}
        else if(line.rfind("XFER:",0)==0){string lst=line.substr(5);stringstream ss(lst);string tok;vector<string> files;while(getline(ss,tok,','))if(!tok.empty())files.push_back(tok);
            int total=(int)files.size(),cnt=0;size_t total_bytes=0;int data_port=g_ctrl_port+1;
            cout<<"[Slave] Transfer started — "<<total<<" files via data port "<<data_port<<endl;
            SOCKET ds=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
            sockaddr_in dsa{};dsa.sin_family=AF_INET;dsa.sin_port=htons(data_port);inet_pton(AF_INET,g_master_ip.c_str(),&dsa.sin_addr);
            {int rtry=0;while(connect(ds,(sockaddr*)&dsa,sizeof(dsa))!=0){if(++rtry>30){cerr<<"[Slave] Data port connect timeout"<<endl;closesocket(ds);goto xfer_done_ctrl;}this_thread::sleep_for(chrono::milliseconds(200));}}
            for(auto& f:files){size_t u=f.rfind('_');string sn=f.substr(0,u);int idx=stoi(f.substr(u+1));stringstream fn;fn<<setw(2)<<setfill('0')<<idx;string path=g_save_dir+"/calib_cam_"+sn+"_"+fn.str()+".jpg";
                ifstream in(path,ios::binary|ios::ate);if(in){size_t sz=in.tellg();in.seekg(0);vector<char> data(sz);in.read(data.data(),sz);in.close();
                    uint32_t sz_be=htonl((uint32_t)sz);
                    if(!sendExact(ds,&sz_be,4)||!sendExact(ds,data.data(),sz)){cerr<<"[Slave] sendExact FAILED for "<<sn<<"_"<<fn.str()<<endl;break;}
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
    string pid=c["participant_id"].as<string>();   // cam_calib.yaml 自带 (P001 / D001 系列)
    g_save_dir=base_dir+"/"+pid+"/pictures"; fs::create_directories(g_save_dir);
    // 标定 XML 输出目录 (calib_cam_chain 的输出, 与 calib_cam_chain.cpp 的计算一致)
    {
        string xpid=pid; try{xpid=c["input_participant_id"].as<string>();if(xpid.empty())xpid=pid;}catch(...){}
        g_xml_dir=base_dir+"/"+xpid+"/output";
    }
    bool test_xfer=false; try{test_xfer=c["test_transfer"].as<bool>();}catch(...){}
    string test_recv; try{test_recv=c["test_transfer_recv_dir"].as<string>();test_recv+="/"+pid+"/pictures";}catch(...){test_recv="D:/calib_transfer_test/"+pid+"/pictures";}
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
                set<string> lf=scanFiles(test_recv);vector<string> mis;for(auto& f:sf)if(!lf.count(f))mis.push_back(f);
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
                    auto& f=mis[file_idx-1];size_t u=f.rfind('_');string sn=f.substr(0,u);int idx=stoi(f.substr(u+1));stringstream fn;fn<<setw(2)<<setfill('0')<<idx;
                    string path=test_recv+"/calib_cam_"+sn+"_"+fn.str()+".jpg";ofstream out(path,ios::binary);out.write(buf.data(),sz);
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
    g_capture_count=getNextCounter(g_save_dir);cout<<"[System] Existing captures: "<<g_capture_count<<endl;g_ready_time=chrono::steady_clock::now();

    while(global_running){
        auto now=chrono::steady_clock::now();bool nu=(now-lui)>=uii;
        // UI
        if(nu){cv::Mat cv;if(g_fault_active.load()){cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);cv::putText(cv,"CAMERA FAULT",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,1.2,cv::Scalar(0,0,255),2);}
        else{cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);int sel=g_enlarged_cam.load();renderGrid(cv,sel);renderEnlarged(cv,sel);cv::line(cv,cv::Point(g_left_w,0),cv::Point(g_left_w,g_win_h),cv::Scalar(60,60,60),2);
            string hints=g_enable_net_sync&&!g_is_master?"[SPACE][z][c][t][ESC/q] disabled (Slave)":"[SPACE] capture  [z] undo  [c] clear  [t] transfer+calib  [ESC/q] quit";cv::putText(cv,hints,cv::Point(g_right_x+10,g_win_h-45),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(140,140,140),1,cv::LINE_AA);
            string cnt="Captures: "+to_string(g_capture_count);cv::putText(cv,cnt,cv::Point(g_right_x+g_right_w-200,35),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,215,255),2,cv::LINE_AA);
            // Bottom-right: save dir + XML dir (右对齐)
            string sdir="Save: "+g_save_dir;cv::Size ssz=cv::getTextSize(sdir,cv::FONT_HERSHEY_SIMPLEX,0.35,1,0);
            cv::putText(cv,sdir,cv::Point(g_right_x+g_right_w-ssz.width-10,g_win_h-35),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(140,140,140),1,cv::LINE_AA);
            string xdir="XML : "+g_xml_dir;cv::Size xsz=cv::getTextSize(xdir,cv::FONT_HERSHEY_SIMPLEX,0.35,1,0);
            cv::putText(cv,xdir,cv::Point(g_right_x+g_right_w-xsz.width-10,g_win_h-20),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(140,140,140),1,cv::LINE_AA);}
        cv::imshow("Calib Capture",cv);lui=now;}

        char key=(char)cv::waitKey(1);
        if(key==27||key=='q'){if(g_enable_net_sync&&g_is_master){sendLine(g_ctrl_sock,"SHUTDOWN");this_thread::sleep_for(chrono::milliseconds(200));}if(!g_enable_net_sync||g_is_master||g_fault_active.load())global_running=false;}
        else if(g_fault_active.load()){}
        else if(key==' '){if(g_enable_net_sync&&!g_is_master){}else{int ctr=getNextCounter(g_save_dir);stringstream ss;ss<<setw(2)<<setfill('0')<<ctr;if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"PHOTO:"+to_string(ctr));g_last_idx.store(ctr);g_capture_count++;
            cout<<"\n[Photo] Capturing index "<<ctr<<endl;
            for(auto& ctx:cam_ctxs){cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}if(!snap.empty()){cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);string fn=g_save_dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg";cv::imwrite(fn,out);cout<<"  -> Saved "<<fn<<endl;}}}}
        else if((key=='t'||key=='T')&&g_is_master&&g_enable_net_sync){sendLine(g_ctrl_sock,"LIST_REQ");string resp;recvLine(g_ctrl_sock,resp,5000);if(resp.rfind("LIST_RESP:",0)!=0)continue;string lst=resp.substr(10);set<string> sf;stringstream ss(lst);string tok;while(getline(ss,tok,','))if(!tok.empty())sf.insert(tok);set<string> lf=scanFiles(g_save_dir);vector<string> mis;for(auto& f:sf)if(!lf.count(f))mis.push_back(f);if(mis.empty()){cout<<"[Transfer] Already in sync.\n"<<endl;continue;}
            cout<<"\n[Transfer] "<<mis.size()<<" files missing. Starting transfer..."<<endl;string xfer="XFER:";for(auto& f:mis)xfer+=f+",";sendLine(g_ctrl_sock,xfer);
            SOCKET dl=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
            {int opt=1;setsockopt(dl,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
            sockaddr_in dsa{};dsa.sin_family=AF_INET;dsa.sin_port=htons(data_port);dsa.sin_addr.s_addr=INADDR_ANY;
            ::bind(dl,(sockaddr*)&dsa,sizeof(dsa));listen(dl,1);}
            SOCKET ds=accept(dl,nullptr,nullptr);
            int rc=0,fi=0;size_t tb=0;auto t0=chrono::steady_clock::now();
            while(true){uint32_t sz_be;if(!recvExact(ds,&sz_be,4)){cerr<<"[XFER] recvExact size header failed"<<endl;break;}
                size_t sz=ntohl(sz_be);if(sz==0)break;
                if(fi>=(int)mis.size()){cerr<<"[XFER] Too many files!"<<endl;break;}
                vector<char> buf(sz);if(!recvExact(ds,buf.data(),sz)){cerr<<"[XFER] recvExact data failed at file "<<fi<<endl;break;}
                tb+=sz;rc++;fi++;auto& f=mis[fi-1];size_t u=f.rfind('_');string sn=f.substr(0,u);int idx=stoi(f.substr(u+1));stringstream fn;fn<<setw(2)<<setfill('0')<<idx;
                string path=g_save_dir+"/calib_cam_"+sn+"_"+fn.str()+".jpg";ofstream out(path,ios::binary);out.write(buf.data(),sz);}
            closesocket(ds);closesocket(dl);
            {string l;recvLine(g_ctrl_sock,l,10000);} // consume XFER_DONE
            auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();double spd=dt>0?(tb/1048576.0/dt):0;
            cout<<"[Transfer] "<<rc<<" files, "<<fixed<<setprecision(1)<<(tb/1048576.0)<<" MB, "<<dt<<"s, "<<spd<<" MB/s\n"<<endl;
            // 传输完成 → 启动 HALCON 标定链
            launchHalconChain();}
        else if((key=='z'||key=='Z')&&!g_fault_active.load()){int li=g_last_idx.load();if(li<0){cout<<"[Undo] No previous capture to undo."<<endl;continue;}stringstream ss;ss<<setw(2)<<setfill('0')<<li;
            cout<<"\n[Undo] Deleting capture index "<<ss.str()<<endl;
            if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir)){string st=e.path().stem().string();if(st.rfind("calib_cam_",0)==0&&st.length()>=2&&st.substr(st.length()-2)==ss.str())fs::remove(e.path());}if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"UNDO:"+to_string(li));g_last_idx.store(li-1);g_undo_count++;if(g_capture_count>0)g_capture_count--;
            cout<<"[Undo] Done.\n"<<endl;}
        else if((key=='c'||key=='C')&&g_is_master&&g_enable_net_sync){
            cout<<"\n[Clear] Removing local calibration photos..."<<endl;
            if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir))if(e.path().extension()==".jpg")fs::remove(e.path());g_capture_count=0;sendLine(g_ctrl_sock,"CLEAR");string ack;recvLine(g_ctrl_sock,ack,2000);
            cout<<"[Clear] Local photos removed.\n"<<endl;}
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
