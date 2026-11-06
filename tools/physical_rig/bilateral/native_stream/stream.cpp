#include "build_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <dm_motor_interfaces/msg/motor_setpoint.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <array>
using Message=dm_motor_interfaces::msg::MotorSetpoint;
constexpr size_t default_log_buffer_bytes=4*1024*1024;
double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Reference{double q,v;const char* phase;int cycle=-1,level=-1,direction=0;};
struct LongCycle{double start,speed,ramp,leg,distance,delta;int index,level;};
const std::vector<LongCycle> &long_cycles(){
 static const auto cycles=[](){
  std::vector<LongCycle> out;out.reserve(102);double start=.3;
  constexpr std::array<double,6> speeds{.6,1.5,3.,6.,9.,13.2};
  for(int group=0;group<17;++group)for(int level=0;level<6;++level){
   const double speed=speeds[level],ramp=std::max(.25,speed/15),delta=std::min(.6,.5*speed);
   const double leg=2*ramp+.6,distance=speed*(ramp+.6)-delta*.1;
   out.push_back({start,speed,ramp,leg,distance,delta,static_cast<int>(out.size()),level});start+=2*leg;
  }
  return out;
 }();return cycles;
}
double long_motion_end(bool pilot=false){const auto &c=long_cycles()[pilot?5:101];return c.start+2*c.leg;}
Reference long_leg(double t,const LongCycle &c){
 if(t<c.ramp)return {c.speed*(t-c.ramp*std::sin(M_PI*t/c.ramp)/M_PI)/2,c.speed*(1-std::cos(M_PI*t/c.ramp))/2,"ramp"};
 if(t<c.ramp+.6){
  const double u=t-c.ramp,area=c.delta*std::min(.1,std::max(0.,u-.05));
  const bool notch=u>=.05&&u<.15;
  return {c.speed*c.ramp/2+c.speed*u-area,c.speed-(notch?c.delta:0),notch?"step":"steady"};
 }
 const double u=std::min(c.ramp,t-c.ramp-.6);
 return {c.speed*c.ramp/2+c.speed*.6-c.delta*.1+c.speed*(u+c.ramp*std::sin(M_PI*u/c.ramp)/M_PI)/2,c.speed*(1+std::cos(M_PI*u/c.ramp))/2,"brake"};
}
Reference long_reference(double t,double q0,bool pilot=false){
 if(t<.3)return {q0,0,"startup"};
 if(t>=long_motion_end(pilot))return {q0,0,"finish"};
 for(const auto &c:long_cycles())if(t<c.start+2*c.leg){
  const double local=t-c.start;
  const bool positive=local<c.leg;
  const auto x=long_leg(local-(positive?0:c.leg),c);
  return {positive?q0+x.q:q0+c.distance-x.q,positive?x.v:-x.v,x.phase,c.index,c.level,positive?1:-1};
 }
 throw std::runtime_error("Invalid long reference time");
}
Reference sweep(double t,double speed,double ramp){
  constexpr double plateau=.6;
  if(t<ramp)return {speed*(t-ramp*std::sin(M_PI*t/ramp)/M_PI)/2,speed*(1-std::cos(M_PI*t/ramp))/2,"ramp"};
  if(t<ramp+plateau)return {speed*ramp/2+speed*(t-ramp),speed,"steady"};
  const double u=std::min(ramp,t-ramp-plateau);
  return {speed*ramp/2+speed*plateau+speed*(u+ramp*std::sin(M_PI*u/ramp)/M_PI)/2,speed*(1+std::cos(M_PI*u/ramp))/2,"brake"};
}
Reference reference(double t,double q0,double speed,const std::string & protocol="sweep"){
 if(protocol=="long_variable"||protocol=="long_variable_pilot")return long_reference(t,q0,protocol=="long_variable_pilot");
 if(protocol=="release"){
  const double r=std::max(.25,speed/15.),release=.3+r+.6;
  if(t<.3)return {q0,0,"baseline"};
  if(t<.3+r){const auto x=sweep(t-.3,speed,r);return {q0+x.q,x.v,"ramp"};}
  if(t<release)return {q0+speed*r/2+speed*(t-.3-r),speed,"release_steady"};
  // All fields change in one message/frame. The neutral position is unused
  // once P=0, and remains encodable even if the coast crosses a native wrap.
  return {0,0,"release_neutral"};
 }
 const double ramp=std::max(.25,speed/15),leg=2*ramp+.6,distance=speed*(ramp+.6);
 if(t<.3)return {q0,0,"baseline"};
 if(t<.3+leg){const auto r=sweep(t-.3,speed,ramp);return {q0+r.q,r.v,r.phase};}
 if(t<.55+leg)return {q0+distance,0,"turnaround"};
 if(t<.55+2*leg){const auto r=sweep(t-.55-leg,speed,ramp);return {q0+distance-r.q,-r.v,r.phase};}
 return {q0,0,"settle"};
}
Reference directed_reference(double t,double q0,double speed,const std::string &protocol,int direction){
 auto r=reference(t,q0,speed,protocol);
 if(direction==1)return r;  // Preserve the original default stream bit-for-bit.
 // Release's zero-gain neutral target remains exactly zero in either direction.
 if(protocol!="release"||std::string(r.phase)!="release_neutral")r.q=q0+direction*(r.q-q0);
 r.v*=direction;r.direction*=direction;return r;
}
bool valid_envelope(double q0,double distance,int direction){
 const double end=q0+direction*distance;
 return std::isfinite(q0)&&std::isfinite(distance)&&distance>=0&&(direction==1||direction==-1)&&
   std::min(q0,end)>-12.4&&std::max(q0,end)<12.4;
}
int main(int argc,char**argv){
 if(argc==2&&std::string(argv[1])=="--build-info"){
  std::cout<<"{\"source_sha256\":\""<<RIG_SOURCE_SHA256<<"\",\"cmake_sha256\":\""<<RIG_CMAKE_SHA256<<"\",\"compiler_id\":\""<<RIG_COMPILER_ID<<"\",\"compiler_version\":\""<<RIG_COMPILER_VERSION<<"\",\"build_type\":\""<<RIG_BUILD_TYPE<<"\",\"default_log_buffer_bytes\":"<<default_log_buffer_bytes<<"}\n";
  return 0;
 }
 if(argc<8||argc>16||(argc-8)%2){std::cerr<<"Usage: rig_stream q0 speed kp kd client_id commands.jsonl summary.json [--log-buffer-bytes BYTES] [--protocol sweep|release|long_variable|long_variable_pilot] [--axis 1|3] [--direction 1|-1]; stdin GO authorizes only targets, never ENABLE\n";return 2;}
 try{
  const double q0=std::stod(argv[1]),speed=std::stod(argv[2]);
  const double kp=std::stod(argv[3]),kd=std::stod(argv[4]);
  const std::string client=argv[5],output=argv[6],summary=argv[7];
  size_t log_buffer_bytes=default_log_buffer_bytes;
  std::string protocol="sweep";bool buffer_set=false,protocol_set=false,axis_set=false,direction_set=false;
  int axis=3,direction=1;
  for(int i=8;i<argc;i+=2){
   const std::string option=argv[i],value=argv[i+1];
   if(option=="--log-buffer-bytes"&&!buffer_set){
    if(value.empty()||value.find_first_not_of("0123456789")!=std::string::npos)throw std::runtime_error("Invalid log buffer option");
    const auto bytes=std::stoull(value);
    if(bytes<2*1024*1024||bytes>64*1024*1024)throw std::runtime_error("Log buffer must be 2..64 MiB");
    log_buffer_bytes=static_cast<size_t>(bytes);buffer_set=true;
   }else if(option=="--protocol"&&!protocol_set){
    if(value!="sweep"&&value!="release"&&value!="long_variable"&&value!="long_variable_pilot")throw std::runtime_error("Unknown protocol");
    protocol=value;protocol_set=true;
   }else if(option=="--axis"&&!axis_set){
    if(value!="1"&&value!="3")throw std::runtime_error("Axis must be 1 or 3");
    axis=std::stoi(value);axis_set=true;
   }else if(option=="--direction"&&!direction_set){
    if(value!="1"&&value!="+1"&&value!="-1")throw std::runtime_error("Direction must be 1 or -1");
    direction=std::stoi(value);direction_set=true;
   }else throw std::runtime_error("Unknown or repeated stream option");
  }
  const double ramp=std::max(.25,speed/15);
  const bool long_mode=protocol=="long_variable"||protocol=="long_variable_pilot";
  if(long_mode&&(speed!=13.2||!buffer_set||log_buffer_bytes!=64*1024*1024))throw std::runtime_error("long_variable requires speed 13.2 and explicit 64 MiB log buffer");
  const double distance=long_mode?19.476:(protocol=="release"?speed*(ramp/2+.6):speed*(ramp+.6));
  const double release_time=.3+ramp+.6;
  // Reserve50ms for command delivery; the scorer still requires a full5s
  // before STOP, anchored at the first actual zero-PD wire frame.
  const double duration=long_mode?long_motion_end(protocol=="long_variable_pilot")+.5:(protocol=="release"?release_time+5.05:2*(2*ramp+.6)+1.05);
  if(!std::isfinite(kp)||!std::isfinite(kd)||kp<=0||kp>30||kd<=0||kd>1)throw std::runtime_error("Invalid axis MIT gains: kp in (0,30], kd in (0,1]");
  if(!std::isfinite(speed)||speed<=0||speed>13.3||!valid_envelope(q0,distance,direction)||client.size()<16)throw std::runtime_error("Invalid trajectory envelope/identity");
  if(long_mode&&(std::min(q0,q0+direction*distance)<=-12.4+1e-9||std::max(q0,q0+direction*distance)>=12.4-1e-9))throw std::runtime_error("Long reference touches native position boundary");
  if(std::ifstream(output).good()||std::ifstream(summary).good())throw std::runtime_error("Existing output refused");
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("rig_native_target_stream");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  const std::string motor_namespace="/motor_"+std::to_string(axis),target_topic=motor_namespace+"/target";
  auto pub=node->create_publisher<Message>(target_topic,rclcpp::QoS(1).reliable());
  // Allocate/touch storage before READY/GO. Configure filebuf before open;
  // Short logs fit the minimum buffer; long has an explicit 64MiB/160000-row
  // contract (fixed record fields <=384B), checked before GO and each publish.
  std::vector<char> log_buffer(log_buffer_bytes);
  std::ofstream log;log.rdbuf()->pubsetbuf(log_buffer.data(),static_cast<std::streamsize>(log_buffer.size()));
  log.open(output);if(!log)throw std::runtime_error("Cannot open command log");
  log<<std::setprecision(17);std::cout<<std::setprecision(17);
  fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL)|O_NONBLOCK);
  bool ready=false,go=false,done=false,failed=false;std::string input,reason;double start=0,last=0,max_gap=0;uint64_t seq=0;const double boot=mono();
  auto timer=node->create_wall_timer(std::chrono::microseconds(2000),[&](){
   if(!go||done)return;
   const double now=mono(),t=now-start;
   if(last>0){max_gap=std::max(max_gap,now-last);if(now-last>.010){failed=true;done=true;reason="C++ target gap >10ms";return;}}
   if(t>=duration){done=true;return;}
   if(long_mode&&seq>=160000){failed=true;done=true;reason="Long command row cap exceeded";return;}
   const auto r=directed_reference(t,q0,speed,protocol,direction);
   const bool neutral=protocol=="release"&&t>=release_time;
   const double target_kp=neutral?0:kp,target_kd=neutral?0:kd;
   Message m;m.header.stamp=node->now();m.client_id=client;m.request_id=++seq;m.sequence=seq;m.valid_for=.04;m.hold_until_lease_expiry=false;
   m.target.mode=1;m.target.position=r.q;m.target.velocity=r.v;m.target.kp=target_kp;m.target.kd=target_kd;m.target.torque=0;pub->publish(m);
   log<<"{\"time\":"<<now<<",\"elapsed\":"<<t<<",\"axis\":"<<axis<<",\"sequence\":"<<seq<<",\"q\":"<<r.q<<",\"v\":"<<r.v<<",\"kp\":"<<target_kp<<",\"kd\":"<<target_kd<<",\"torque\":0,\"phase\":\""<<r.phase<<"\"";
   if(long_mode)log<<",\"cycle\":"<<r.cycle<<",\"level\":"<<r.level<<",\"direction\":"<<r.direction;
   log<<"}\n";last=now;
  });
  while(rclcpp::ok()&&!done){
   executor.spin_some();
   bool driver_seen=false;if(!ready)for(const auto & endpoint:node->get_subscriptions_info_by_topic(target_topic)){if(endpoint.node_name()=="motor_controller"&&endpoint.node_namespace()==motor_namespace)driver_seen=true;}
   if(!ready&&driver_seen&&pub->get_subscription_count()>=2&&mono()-boot>.5){ready=true;std::cout<<"{\"type\":\"READY\",\"client_id\":\""<<client<<"\"}"<<std::endl;}
   char buffer[128];const auto count=read(STDIN_FILENO,buffer,sizeof(buffer));
   if(count==0){failed=true;reason="Parent pipe EOF";break;}
   if(count>0){input.append(buffer,count);if(input.size()>128)throw std::runtime_error("Input overflow");if(input.find('\n')!=std::string::npos){if(input!="GO\n"||go||!ready)throw std::runtime_error("Invalid GO");go=true;start=mono();std::cout<<"{\"type\":\"START\",\"started\":"<<start<<"}"<<std::endl;input.clear();}}
   if(!go&&mono()-boot>10){failed=true;reason="GO timeout";break;}
   std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  if(!rclcpp::ok()&&!done){failed=true;reason="Interrupted";}
  log.close();if(!log){failed=true;reason="Command log finalization failed";}
  std::ofstream out(summary);out<<std::setprecision(17)<<"{\"success\":"<<(!failed&&done?"true":"false")<<",\"started\":"<<start<<",\"commands\":"<<seq<<",\"max_gap\":"<<max_gap<<",\"axis_id\":"<<axis<<",\"direction\":"<<direction<<",\"driver_kp\":"<<kp<<",\"driver_kd\":"<<kd<<",\"protocol\":\""<<protocol<<"\",\"protocol_version\":"<<(protocol=="release"?2:1)<<",\"log_buffer_bytes\":"<<log_buffer_bytes<<",\"reason\":\""<<reason<<"\"}\n";out.close();
  if(!out){failed=true;reason="Summary finalization failed";std::cerr<<reason<<"\n";}
  std::cout<<"{\"type\":\"DONE\",\"success\":"<<(!failed&&done?"true":"false")<<",\"commands\":"<<seq<<",\"max_gap\":"<<max_gap<<"}"<<std::endl;rclcpp::shutdown();return failed?1:0;
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
