// Start the D435i sensors one by one in a chosen order and report frames per stream, to show
// which start orders hang the camera. Letters: D = depth 848x480@30, C = colour 1280x720@30 RGB8,
// M = motion (accel 100 Hz + gyro 200 Hz).
//
// build: g++ -std=c++14 -O1 -o sensor_order_test sensor_order_test.cpp -lrealsense2 -lpthread
// run:   ./sensor_order_test MCD 1000        open+start each sensor, 1 s apart
//        ./sensor_order_test MCD 0 openall   open all sensors, then start all
// "final: ... C=0 A=1 G=0" means colour and IMU hung; reset with rs_reset.py.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <atomic>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
using namespace std; using namespace chrono;
atomic<int> nD{0},nC{0},nA{0},nG{0};
steady_clock::time_point t0; atomic<long> lastImu{-1};
void cb(rs2::frame f){
  auto s=f.get_profile().stream_type();
  long ms=duration_cast<milliseconds>(steady_clock::now()-t0).count();
  if(s==RS2_STREAM_DEPTH) nD++; else if(s==RS2_STREAM_COLOR) nC++;
  else { if(s==RS2_STREAM_ACCEL) nA++; else nG++; lastImu=ms; }
}
// usage: order <ORDER e.g. MCD> <gap_ms> [openall] [W H Wc Hc]
int main(int argc,char**argv){
  string order=argc>1?argv[1]:"MCD"; int gap=argc>2?atoi(argv[2]):1000;
  bool openall=argc>3 && string(argv[3])=="openall";
  int dw=848,dh=480,cw=1280,ch=720;
  if(argc>7){dw=atoi(argv[4]);dh=atoi(argv[5]);cw=atoi(argv[6]);ch=atoi(argv[7]);}
  rs2::context ctx; auto dev=ctx.query_devices()[0];
  map<char,rs2::sensor> S; map<char,vector<rs2::stream_profile>> P;
  for(auto s:dev.query_sensors()){
    string n=s.get_info(RS2_CAMERA_INFO_NAME);
    char k = n[0]=='S' ? 'D' : (n[0]=='R' ? 'C' : 'M');
    S.emplace(k,s);
    bool a=false,g=false;
    for(auto p:s.get_stream_profiles()){
      if(k=='M'){
        if(!a&&p.stream_type()==RS2_STREAM_ACCEL&&p.fps()==100){P[k].push_back(p);a=true;}
        if(!g&&p.stream_type()==RS2_STREAM_GYRO&&p.fps()==200){P[k].push_back(p);g=true;}
        continue;
      }
      auto v=p.as<rs2::video_stream_profile>();
      if(k=='D'&&p.stream_type()==RS2_STREAM_DEPTH&&v.width()==dw&&v.height()==dh&&p.fps()==30){P[k].push_back(p);break;}
      if(k=='C'&&p.stream_type()==RS2_STREAM_COLOR&&p.format()==RS2_FORMAT_RGB8&&v.width()==cw&&v.height()==ch&&p.fps()==30){P[k].push_back(p);break;}
    }
  }
  t0=steady_clock::now();
  auto rep=[&](string w){
    long ms=duration_cast<milliseconds>(steady_clock::now()-t0).count();
    cout<<"  t="<<ms<<"ms "<<w<<": D="<<nD<<" C="<<nC<<" A="<<nA<<" G="<<nG<<" lastIMU@"<<lastImu<<endl; };
  if(openall){
    for(char k:order){ S.at(k).open(P[k]); rep(string("opened ")+k); }
    for(char k:order){ S.at(k).start(cb); rep(string("started ")+k); }
  } else {
    for(char k:order){
      S.at(k).open(P[k]); S.at(k).start(cb); rep(string("opened+started ")+k);
      this_thread::sleep_for(milliseconds(gap)); rep(string("after gap ")+k);
    }
  }
  this_thread::sleep_for(seconds(3)); rep("final");
  for(char k:order){ S.at(k).stop(); S.at(k).close(); }
  return 0;
}
