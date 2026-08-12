// test_calib_transfer.cpp — TCP-based calibration image capture + transfer
// Replaces UDP with a single TCP connection for all communication.
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
int g_win_w=1600, g_win_h=800, g_left_w, g_right_x, g_right_w, g_thumb_w, g_thumb_h;
atomic<int> g_enlarged_cam{-1};
SOCKET g_ctrl_sock = INVALID_SOCKET;  // TCP for all communication
SOCKET g_listen_sock = INVALID_SOCKET;
atomic<bool> g_fault_active{false}; atomic<int> g_faulty_cam{-1};
atomic<bool> g_fault_on_master{false};
chrono::steady_clock::time_point g_fault_time, g_ready_time;
atomic<int> g_capture_count{-1}, g_last_idx{-1}; int g_undo_count=0;

// ================== TCP helpers ==================
bool recvLine(SOCKET s, string& line, int timeout_ms=3000) {
    DWORD to=timeout_ms; setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,(const char*)&to,sizeof(to));
    static thread_local string leftover;
    size_t nl=leftover.find('\n');
    if(nl!=string::npos){line=leftover.substr(0,nl);if(!line.empty()&&line.back()=='\r')line.pop_back();leftover=leftover.substr(nl+1);return true;}
    char buf[256]; auto dl=chrono::steady_clock::now()+chrono::milliseconds(timeout_ms);
    while(chrono::steady_clock::now()<dl){int n=recv(s,buf,sizeof(buf)-1,0);if(n<=0)return false;buf[n]='\0';leftover+=buf;
        nl=leftover.find('\n');if(nl!=string::npos){line=leftover.substr(0,nl);if(!line.empty()&&line.back()=='\r')line.pop_back();leftover=leftover.substr(nl+1);return true;}}
    return false;
}
bool sendLine(SOCKET s,const string& m){string d=m+"\n";return send(s,d.c_str(),(int)d.length(),0)>0;}
bool recvExact(SOCKET s,void* buf,size_t n){size_t got=0;while(got<n){int r=recv(s,(char*)buf+got,(int)(n-got),0);if(r<=0)return false;got+=r;}return true;}

// ================== Camera ==================
struct CameraContext {string sn; BaslerCamera cam{""}; bool is_mono=true; thread capture_thread,copy_thread; atomic<bool> running{true};
    cv::Mat latest_frame; FrameMeta latest_meta; mutex frame_mtx;
    queue<pair<Pylon::CBaslerUniversalGrabResultPtr,FrameMeta>> copy_queue; mutex copy_mtx; condition_variable copy_cv;
    atomic<int64_t> last_block_id{-1}; atomic<chrono::steady_clock::time_point> last_frame_time{chrono::steady_clock::now()}; atomic<bool> has_streamed{false};
    explicit CameraContext(string s):sn(s),cam(s){}};
vector<shared_ptr<CameraContext>> cam_ctxs;

void copyWorker(shared_ptr<CameraContext> ctx){while(ctx->running){pair<Pylon::CBaslerUniversalGrabResultPtr,FrameMeta> t;
    {unique_lock<mutex> lk(ctx->copy_mtx);ctx->copy_cv.wait(lk,[&]{return !ctx->copy_queue.empty()||!ctx->running;});if(!ctx->running&&ctx->copy_queue.empty())break;t=ctx->copy_queue.front();ctx->copy_queue.pop();}
    cv::Mat tmp(t.first->GetHeight(),t.first->GetWidth(),CV_8UC1,t.first->GetBuffer());cv::Mat c=tmp.clone();{lock_guard<mutex> lk(ctx->frame_mtx);ctx->latest_frame=c;ctx->latest_meta=t.second;}
    ctx->last_block_id.store(t.second.blockID,memory_order_relaxed);ctx->last_frame_time.store(chrono::steady_clock::now(),memory_order_relaxed);ctx->has_streamed.store(true,memory_order_relaxed);}}

void captureWorker(shared_ptr<CameraContext> ctx,double fps,double gain,double gamma,double exp,double me){
    if(!ctx->cam.open(TriggerMode::Software))return;ctx->is_mono=ctx->cam.isMono();if(ctx->is_mono&&me>1.0){exp*=me;fps/=me;}
    try{ctx->cam.setFrameRate(fps);ctx->cam.setGain(gain);ctx->cam.setGamma(gamma);ctx->cam.setExposureTime(exp);}catch(...){}
    ctx->cam.setFrameCallback([ctx](const Pylon::CBaslerUniversalGrabResultPtr& p,FrameMeta m){lock_guard<mutex> lk(ctx->copy_mtx);if(ctx->copy_queue.size()<2){ctx->copy_queue.push({p,m});ctx->copy_cv.notify_one();}});
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
            for(auto& ctx:cam_ctxs){cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}if(!snap.empty()){cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);cv::imwrite(g_save_dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg",out);}}}
        else if(line.rfind("UNDO:",0)==0){int idx=stoi(line.substr(5));stringstream ss;ss<<setw(2)<<setfill('0')<<idx;if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir)){string st=e.path().stem().string();if(st.rfind("calib_cam_",0)==0&&st.length()>=2&&st.substr(st.length()-2)==ss.str())fs::remove(e.path());}g_last_idx.store(idx-1);if(g_capture_count>0)g_capture_count--;}
        else if(line=="LIST_REQ"){stringstream fl;for(auto& f:scanFiles(g_save_dir))fl<<f<<",";sendLine(s,"LIST_RESP:"+fl.str());}
        else if(line=="CLEAR"){if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir))if(e.path().extension()==".jpg")fs::remove(e.path());g_capture_count=0;g_last_idx=-1;sendLine(s,"CLEAR_DONE");}
        else if(line.rfind("XFER:",0)==0){string lst=line.substr(5);stringstream ss(lst);string tok;vector<string> files;while(getline(ss,tok,','))if(!tok.empty())files.push_back(tok);
            cout<<"[Slave] XFER: sending "<<files.size()<<" files"<<endl;
            for(auto& f:files){size_t u=f.rfind('_');string sn=f.substr(0,u);int idx=stoi(f.substr(u+1));stringstream fn;fn<<setw(2)<<setfill('0')<<idx;string path=g_save_dir+"/calib_cam_"+sn+"_"+fn.str()+".jpg";
                ifstream in(path,ios::binary|ios::ate);if(in){size_t sz=in.tellg();in.seekg(0);vector<char> data(sz);in.read(data.data(),sz);in.close();
                    char hdr[128];snprintf(hdr,sizeof(hdr),"FILE:%s:%d:%zu",sn.c_str(),idx,sz);sendLine(s,hdr);send(s,data.data(),(int)sz,0);
                    cout<<"[Slave] Sent "<<sn<<"_"<<fn.str()<<" ("<<sz<<" bytes)"<<endl;}
                else{cerr<<"[Slave] Cannot read "<<path<<endl;}}
            sendLine(s,"XFER_DONE");cout<<"[Slave] XFER_DONE sent."<<endl;}
        else if(line.rfind("FAULT:",0)==0&&!g_fault_active.load()){/*handle fault*/}
        else if(line=="SHUTDOWN"){global_running=false;}
    }
}

// ================== main ==================
int main(){
    cout<<"=== Calibration Image Capture (TCP) ==="<<endl;
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2),&wsa);
#endif
    Cfg cfg("cfg/cam_calib.yaml"); auto& c=cfg["calib"];
    g_is_master=c["is_master"].as<bool>(); string mip=c["master_ip"].as<string>(),sip=c["slave_ip"].as<string>();
    int port=c["port"].as<int>(); g_enable_net_sync=c["enable_net_sync"].as<bool>();
    vector<string> sns=c["cam_indices"].as<vector<string>>();
    string base_dir=c["calib_save_dir"].as<string>();
    string pid="P001"; try{Cfg cfg_cap("cfg/capture.yaml");pid=cfg_cap["capture"]["participant_id"].as<string>();}catch(...){}
    g_save_dir=base_dir+"/"+pid+"/pictures"; fs::create_directories(g_save_dir);
    bool test_xfer=false; try{test_xfer=c["test_transfer"].as<bool>();}catch(...){}
    string test_recv; try{test_recv=c["test_transfer_recv_dir"].as<string>();test_recv+="/"+pid+"/pictures";}catch(...){test_recv="D:/calib_transfer_test/"+pid+"/pictures";}
    double fps=c["fps"].as<double>(),gain=c["gain"].as<double>(),gamma=c["gamma"].as<double>(),exp=c["exposure_time"].as<double>(),me=c["calib_mono_exp_ext"].as<double>();
    g_win_w=c["window_width"].as<int>(); g_win_h=c["window_height"].as<int>(); double uif=c["ui_fps"].as<double>();
    cout<<"[Init] Config loaded. Role="<<(g_is_master?"MASTER":"SLAVE")<<" Port="<<port<<" TestXfer="<<(test_xfer?"ON":"OFF")<<endl;
    cout<<"[Init] net_sync="<<g_enable_net_sync<<" is_master="<<g_is_master<<endl;

    // ---- TCP setup ----
    thread cmd_thread;
    cout<<"[TCP] Entering setup block..."<<endl;
    if(g_enable_net_sync){
        if(g_is_master){ g_listen_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); int opt=1;setsockopt(g_listen_sock,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
            sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(port);sa.sin_addr.s_addr=INADDR_ANY;::bind(g_listen_sock,(sockaddr*)&sa,sizeof(sa));listen(g_listen_sock,1);
            cout<<"[TCP] Master listening ::"<<port<<endl; sockaddr_in ca;socklen_t cl=sizeof(ca);g_ctrl_sock=accept(g_listen_sock,(sockaddr*)&ca,&cl);
            string hl; recvLine(g_ctrl_sock,hl,10000); if(hl=="READY") sendLine(g_ctrl_sock,"ACK"); else{cerr<<"[TCP] Handshake fail"<<endl;return 1;}
            cout<<"[TCP] Slave connected + handshake OK."<<endl;}
        else{ g_ctrl_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(port);inet_pton(AF_INET,mip.c_str(),&sa.sin_addr);
            cout<<"[TCP] Slave connecting to "<<mip<<":"<<port<<"..."<<endl;
            int retry=0; while(connect(g_ctrl_sock,(sockaddr*)&sa,sizeof(sa))!=0){if(++retry%10==1)cerr<<"[TCP] Connect retry #"<<retry<<"..."<<endl;this_thread::sleep_for(chrono::milliseconds(500));}
            sendLine(g_ctrl_sock,"READY"); string hl; recvLine(g_ctrl_sock,hl,10000); if(hl!="ACK"){cerr<<"[TCP] Handshake fail"<<endl;return 1;}
            cout<<"[TCP] Connected to Master + handshake OK."<<endl;cmd_thread=thread(slaveCmdWorker,g_ctrl_sock);}
    }

    // ---- test_transfer mode (skip cameras, show transfer UI) ----
    if(test_xfer){
        cout<<"[Mode] Test transfer — no cameras, transfer UI only."<<endl;
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
                cv::imshow("Calib Transfer Test",cv);lui=now;}
            char key=(char)cv::waitKey(30);
            if(key=='q'||key==27){if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"SHUTDOWN");global_running=false;}
            else if((key=='c'||key=='C')&&g_is_master){xfer_cnt=0;xfer_bytes=0;xfer_total=0;xfer_done=false;xfer_status="";if(fs::exists(test_recv))for(auto& e:fs::directory_iterator(test_recv))if(e.path().extension()==".jpg")fs::remove(e.path());cout<<"[Clear] Master test dir cleared. Slave untouched."<<endl;}
            else if((key=='t'||key=='T')&&g_is_master&&g_enable_net_sync){
                xfer_status="Transferring...";xfer_cnt=0;xfer_bytes=0;xfer_done=false;
                cout<<"[DBG] Sending LIST_REQ..."<<endl;sendLine(g_ctrl_sock,"LIST_REQ");
                string resp; if(!recvLine(g_ctrl_sock,resp,5000)){cerr<<"[DBG] LIST_REQ timeout"<<endl;continue;}
                cout<<"[DBG] LIST_RESP len="<<resp.length()<<" preview="<<resp.substr(0,min(80,(int)resp.length()))<<endl;
                if(resp.rfind("LIST_RESP:",0)!=0){xfer_status="LIST_REQ failed";continue;}
                string lst=resp.substr(10);set<string> sf;stringstream ss(lst);string tok;while(getline(ss,tok,','))if(!tok.empty())sf.insert(tok);
                cout<<"[DBG] Slave has "<<sf.size()<<" files."<<endl;
                set<string> lf=scanFiles(test_recv);vector<string> mis;for(auto& f:sf)if(!lf.count(f))mis.push_back(f);
                cout<<"[DBG] Master has "<<lf.size()<<" files, missing "<<mis.size()<<endl;
                if(mis.empty()){xfer_status="Already in sync.";xfer_done=true;continue;}
                xfer_total=(int)mis.size();xfer_status="Transferring "+to_string(xfer_total)+" files...";
                string xfer="XFER:";for(auto& f:mis)xfer+=f+",";cout<<"[DBG] Sending XFER with "<<mis.size()<<" files..."<<endl;sendLine(g_ctrl_sock,xfer);
                auto t0=chrono::steady_clock::now();
                while(true){string l;if(!recvLine(g_ctrl_sock,l,30000)){cerr<<"[DBG] recv timeout/error in XFER loop (got "<<xfer_cnt<<" files)"<<endl;break;}if(l=="XFER_DONE"){cout<<"[DBG] Received XFER_DONE"<<endl;break;}if(l.rfind("FILE:",0)==0){stringstream fs(l.substr(5));string sn,ix,sz;getline(fs,sn,':');getline(fs,ix,':');getline(fs,sz);size_t s=(size_t)stoull(sz);cout<<"[DBG] FILE "<<sn<<"_"<<ix<<" size="<<s<<endl;vector<char> buf(s);if(!recvExact(g_ctrl_sock,buf.data(),s)){cerr<<"[DBG] recvExact FAILED for "<<sn<<"_"<<ix<<endl;break;}xfer_bytes+=s;xfer_cnt++;stringstream fn;fn<<setw(2)<<setfill('0')<<stoi(ix);string path=test_recv+"/calib_cam_"+sn+"_"+fn.str()+".jpg";ofstream out(path,ios::binary);out.write(buf.data(),s);
                    auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();xfer_speed=dt>0?(xfer_bytes/1048576.0)/dt:0;}else{cout<<"[DBG] Unknown recv: "<<l.substr(0,min(60,(int)l.length()))<<endl;}}
                auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();xfer_speed=dt>0?(xfer_bytes/1048576.0)/dt:0;xfer_done=true;
                cout<<"[XFER] "<<xfer_cnt<<" files, "<<fixed<<setprecision(1)<<(xfer_bytes/1048576.0)<<" MB, "<<dt<<"s, "<<xfer_speed<<" MB/s"<<endl;
            }
        }
        if(cmd_thread.joinable())cmd_thread.join();
        if(g_listen_sock!=INVALID_SOCKET)closesocket(g_listen_sock);if(g_ctrl_sock!=INVALID_SOCKET)closesocket(g_ctrl_sock);
        cv::destroyAllWindows();WSACleanup();return 0;
    }

    // ---- Camera init (non-test mode) ----
    Pylon::PylonInitialize();
    for(size_t i=0;i<sns.size();++i){auto ctx=make_shared<CameraContext>(sns[i]);cam_ctxs.push_back(ctx);}
    for(auto& ctx:cam_ctxs){ctx->running=true;ctx->copy_thread=thread(copyWorker,ctx);ctx->capture_thread=thread(captureWorker,ctx,fps,gain,gamma,exp,me);}

    // ---- UI ----
    cv::namedWindow("Calib Capture",cv::WINDOW_NORMAL);cv::resizeWindow("Calib Capture",g_win_w,g_win_h);updateLayout();cv::setMouseCallback("Calib Capture",onMouse);
    auto uii=chrono::milliseconds((int)(1000.0/uif));auto lui=chrono::steady_clock::now()-uii;
    g_capture_count=getNextCounter(g_save_dir);cout<<"[Init] Existing captures: "<<g_capture_count<<endl;g_ready_time=chrono::steady_clock::now();

    while(global_running){
        auto now=chrono::steady_clock::now();bool nu=(now-lui)>=uii;
        // Health check
        if(!g_fault_active.load()){for(size_t i=0;i<cam_ctxs.size();++i){if(!cam_ctxs[i]->has_streamed.load())continue;if(chrono::duration<double>(now-cam_ctxs[i]->last_frame_time.load()).count()>1.0){cerr<<"[FAULT] Cam "<<cam_ctxs[i]->sn<<" stalled!"<<endl;g_fault_active.store(true);g_faulty_cam.store((int)i);g_fault_on_master.store(g_is_master);if(g_enable_net_sync)sendLine(g_ctrl_sock,"FAULT:"+string(g_is_master?"M":"S")+to_string(i));for(auto& c:cam_ctxs){c->running=false;c->copy_cv.notify_all();}for(auto& c:cam_ctxs){if(c->capture_thread.joinable())c->capture_thread.join();if(c->copy_thread.joinable())c->copy_thread.join();}break;}}}
        // UI
        if(nu){cv::Mat cv;if(g_fault_active.load()){cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);cv::putText(cv,"CAMERA FAULT",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,1.2,cv::Scalar(0,0,255),2);}
        else{cv=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);int sel=g_enlarged_cam.load();renderGrid(cv,sel);renderEnlarged(cv,sel);cv::line(cv,cv::Point(g_left_w,0),cv::Point(g_left_w,g_win_h),cv::Scalar(60,60,60),2);
            string hints=g_enable_net_sync&&!g_is_master?"[SPACE][z][c][t][ESC/q] disabled (Slave)":"[SPACE] capture  [z] undo  [c] clear  [t] transfer  [ESC/q] quit";cv::putText(cv,hints,cv::Point(g_right_x+10,g_win_h-45),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(140,140,140),1,cv::LINE_AA);
            string cnt="Captures: "+to_string(g_capture_count);cv::putText(cv,cnt,cv::Point(g_right_x+g_right_w-200,35),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,215,255),2,cv::LINE_AA);}
        cv::imshow("Calib Capture",cv);lui=now;}

        char key=(char)cv::waitKey(1);
        if(key==27||key=='q'){if(g_enable_net_sync&&g_is_master){sendLine(g_ctrl_sock,"SHUTDOWN");this_thread::sleep_for(chrono::milliseconds(200));}if(!g_enable_net_sync||g_is_master||g_fault_active.load())global_running=false;}
        else if(g_fault_active.load()){}
        else if(key==' '){if(g_enable_net_sync&&!g_is_master){}else{int ctr=getNextCounter(g_save_dir);stringstream ss;ss<<setw(2)<<setfill('0')<<ctr;if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"PHOTO:"+to_string(ctr));g_last_idx.store(ctr);g_capture_count++;cout<<"[Photo] "<<ctr<<endl;
            for(auto& ctx:cam_ctxs){cv::Mat snap;{lock_guard<mutex> lk(ctx->frame_mtx);snap=ctx->latest_frame.clone();}if(!snap.empty()){cv::Mat out;if(ctx->is_mono)out=snap.clone();else cv::cvtColor(snap,out,cv::COLOR_BayerRG2RGB);cv::imwrite(g_save_dir+"/calib_cam_"+ctx->sn+"_"+ss.str()+".jpg",out);}}}}
        else if((key=='t'||key=='T')&&g_is_master&&g_enable_net_sync){sendLine(g_ctrl_sock,"LIST_REQ");string resp;recvLine(g_ctrl_sock,resp,5000);if(resp.rfind("LIST_RESP:",0)!=0)continue;string lst=resp.substr(10);set<string> sf;stringstream ss(lst);string tok;while(getline(ss,tok,','))if(!tok.empty())sf.insert(tok);set<string> lf=scanFiles(g_save_dir);vector<string> mis;for(auto& f:sf)if(!lf.count(f))mis.push_back(f);if(mis.empty()){cout<<"[XFER] Already in sync."<<endl;continue;}
            cout<<"[XFER] Transferring "<<mis.size()<<" files..."<<endl;string xfer="XFER:";for(auto& f:mis)xfer+=f+",";sendLine(g_ctrl_sock,xfer);
            int rc=0;size_t tb=0;auto t0=chrono::steady_clock::now();while(true){string l;if(!recvLine(g_ctrl_sock,l,30000))break;if(l=="XFER_DONE")break;if(l.rfind("FILE:",0)==0){stringstream fs(l.substr(5));string sn,ix,sz;getline(fs,sn,':');getline(fs,ix,':');getline(fs,sz);size_t s=(size_t)stoull(sz);vector<char> buf(s);recvExact(g_ctrl_sock,buf.data(),s);tb+=s;rc++;stringstream fn;fn<<setw(2)<<setfill('0')<<stoi(ix);string path=g_save_dir+"/calib_cam_"+sn+"_"+fn.str()+".jpg";ofstream out(path,ios::binary);out.write(buf.data(),s);}}
            auto dt=chrono::duration<double>(chrono::steady_clock::now()-t0).count();cout<<"[XFER] Done: "<<rc<<" files, "<<fixed<<setprecision(1)<<(tb/1048576.0)<<" MB, "<<dt<<"s, "<<(tb/1048576.0/dt)<<" MB/s"<<endl;}
        else if((key=='z'||key=='Z')&&!g_fault_active.load()){int li=g_last_idx.load();if(li<0){cout<<"[Undo] Nothing to undo."<<endl;continue;}stringstream ss;ss<<setw(2)<<setfill('0')<<li;if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir)){string st=e.path().stem().string();if(st.rfind("calib_cam_",0)==0&&st.length()>=2&&st.substr(st.length()-2)==ss.str())fs::remove(e.path());}if(g_enable_net_sync&&g_is_master)sendLine(g_ctrl_sock,"UNDO:"+to_string(li));g_last_idx.store(li-1);g_undo_count++;if(g_capture_count>0)g_capture_count--;}
        else if((key=='c'||key=='C')&&g_is_master&&g_enable_net_sync){if(fs::exists(g_save_dir))for(auto& e:fs::directory_iterator(g_save_dir))if(e.path().extension()==".jpg")fs::remove(e.path());g_capture_count=0;sendLine(g_ctrl_sock,"CLEAR");string ack;recvLine(g_ctrl_sock,ack,2000);cout<<"[Clear] Done."<<endl;}
    }

    for(auto& ctx:cam_ctxs){ctx->running=false;ctx->copy_cv.notify_all();if(ctx->capture_thread.joinable())ctx->capture_thread.join();if(ctx->copy_thread.joinable())ctx->copy_thread.join();}
    if(cmd_thread.joinable())cmd_thread.join();
    if(g_listen_sock!=INVALID_SOCKET)closesocket(g_listen_sock);if(g_ctrl_sock!=INVALID_SOCKET)closesocket(g_ctrl_sock);
    cv::destroyAllWindows();Pylon::PylonTerminate();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
