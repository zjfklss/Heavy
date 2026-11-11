#include "rm_sim/adapters/external.hpp"
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <sstream>

namespace rm_sim::adapters {
using nlohmann::json;
namespace {
void keys(const json& value,std::initializer_list<const char*> allowed) {
  ensure(value.is_object(),"Expected external adapter JSON object");
  std::set<std::string> names;for(auto key:allowed)names.insert(key);
  for(auto it=value.begin();it!=value.end();++it)
    if(!names.count(it.key()))throw std::invalid_argument("Unknown adapter field: "+it.key());
}
json vector(Vec3 p){return {p.x,p.y,p.z};}
json pose(const Pose& p){return {{"translation",vector(p.translation)},
  {"rotation",{vector(p.rotation.x),vector(p.rotation.y),vector(p.rotation.z)}}};}
std::string session_id(){
  std::random_device rng;std::ostringstream out;
  for(int i=0;i<4;++i)out<<std::hex<<std::setfill('0')<<std::setw(8)<<rng();
  return out.str();
}
int milliseconds(double seconds){return int(std::llround(seconds*1000));}
void same(const json& header,const char* key,const json& expected) {
  ensure(header.contains(key) && header.at(key)==expected &&
    (!expected.is_number_integer() || header.at(key).is_number_unsigned()),"External adapter response/session/sequence mismatch");
}
void empty_payload(const Message& message){ensure(message.payload.empty(),"Control responses must not contain binary payload");}
double angle(const json& value) {
  ensure(value.is_number(),"External control angle must be a number");
  double result=value.get<double>();
  ensure(std::isfinite(result) && std::abs(result)<=1e6,"Invalid external control angle");return result;
}
}
void ExternalConfig::validate(const RunnerConfig& cfg) const {
  profile(adapter);validate_endpoint(host,port);
  for(auto seconds:{connect_timeout,step_timeout})
    ensure(std::isfinite(seconds) && seconds>=.001 && seconds<=60,"Adapter timeout must be in [0.001,60] seconds");
  ensure(cfg.sensor.enabled && cfg.sensor.output=="image","External adapters require an enabled BGR image camera");
  ensure(cfg.actuator.feedback_enabled,"External adapters require enabled actuator feedback");
  double batch=std::ceil(cfg.sensor.frequency/cfg.algorithm_frequency)+1;
  ensure(batch<=1024 && std::ceil(cfg.actuator.feedback_frequency/cfg.algorithm_frequency)+1<=1024,
    "External message batch exceeds 1024 samples; increase algorithm frequency");
  ensure(batch*cfg.sensor.calibration.width*cfg.sensor.calibration.height*3<=max_payload_bytes,
    "External image batch exceeds 64 MiB; increase algorithm frequency or reduce image size");
}
ExternalConfig parse_config(const json& algorithm) {
  keys(algorithm,{"kind","adapter","host","port","connect_timeout","step_timeout"});
  ExternalConfig cfg;
  cfg.adapter=algorithm.value("adapter",cfg.adapter);cfg.host=algorithm.value("host",cfg.host);
  if(algorithm.contains("port")) {
    const auto& port=algorithm["port"];
    ensure(port.is_number_integer() && port>=1 && port<=65535,"Invalid adapter TCP port");cfg.port=port.get<int>();
  }
  cfg.connect_timeout=algorithm.value("connect_timeout",cfg.connect_timeout);
  cfg.step_timeout=algorithm.value("step_timeout",cfg.step_timeout);return cfg;
}
json encode_config(const ExternalConfig& c){return {{"kind","external"},{"adapter",c.adapter},{"host",c.host},
  {"port",c.port},{"connect_timeout",c.connect_timeout},{"step_timeout",c.step_timeout}};}
ExternalAlgorithm::ExternalAlgorithm(ExternalConfig config,RunnerConfig context,Tick duration,bool realtime)
 :config_(std::move(config)),context_(std::move(context)),duration_(duration),realtime_(realtime){config_.validate(context_);}
std::string ExternalAlgorithm::version()const{return "external-tcp/1:"+config_.adapter;}
void ExternalAlgorithm::start() {
  ensure(!started_,"External adapter session is single-use");
  if(!realtime_)std::cout<<"[adapter] connecting "<<config_.adapter<<" "<<config_.host<<":"<<config_.port<<std::endl;
  auto deadline=deadline_after(milliseconds(config_.connect_timeout));
  connection_=Connection::connect(config_.host,config_.port,deadline);
  session_=session_id();const auto& c=context_;const auto& k=c.sensor.calibration;const auto& a=c.actuator;
  json hello={{"protocol",wire_version},{"type","hello"},{"session_id",session_},{"adapter",config_.adapter},
    {"units",{{"length","m"},{"time","s"},{"angle","rad"},{"world_axes","x-forward,y-left,z-up"},
      {"camera_axes","x-right,y-down,z-forward"},{"yaw_left_positive",true},
      {"pitch_up_sign",profile(config_.adapter).pitch_sign}}},
    {"configuration",{{"timestep",c.world.timestep},{"duration",duration_*c.world.timestep},
      {"algorithm_frequency",c.algorithm_frequency},{"gravity",vector(c.world.gravity)},
      {"camera",{{"id",c.sensor.id},{"frequency",c.sensor.frequency},{"latency",c.sensor.latency},
        {"enemy_color",c.sensor.image.color},
        {"calibration",{{"id",k.id},{"width",k.width},{"height",k.height},{"fx",k.fx},{"fy",k.fy},{"cx",k.cx},{"cy",k.cy}}}}},
      {"actuator",{{"base_pose",pose(a.base_pose)},{"yaw_offset",vector(a.yaw_offset)},
        {"pitch_offset",vector(a.pitch_offset)},{"camera_mount",pose(a.camera_mount)},{"muzzle_mount",pose(a.muzzle_mount)},
        {"nominal_muzzle_speed",a.muzzle_speed},{"command_latency",a.command_latency},{"fire_latency",a.fire_latency},
        {"fire_interval",a.fire_interval},{"feedback_frequency",a.feedback_frequency},{"feedback_latency",a.feedback_latency}}}}}};
  if(realtime_){hello["configuration"]["realtime"]=true;hello["configuration"]["duration"]=nullptr;}
  connection_.send({hello,{}},deadline);
  auto reply=connection_.receive(deadline);empty_payload(reply);
  keys(reply.header,{"protocol","type","session_id","adapter","peer_version"});
  same(reply.header,"type","hello_ack");same(reply.header,"session_id",session_);same(reply.header,"adapter",config_.adapter);
  peer_version_=reply.header.at("peer_version").get<std::string>();
  ensure(!peer_version_.empty() && peer_version_.size()<=128,"Invalid external peer version");
  started_=true;
  std::cout<<"[adapter] connected "<<peer_version_<<std::endl;
}
std::vector<ControlRequest> ExternalAlgorithm::update(const AlgorithmInput& input) {
  ensure(started_ && !finished_,"External adapter session is not active");
  Message step;
  step.header={{"protocol",wire_version},{"type","step"},{"session_id",session_},{"sequence",sequence_},
    {"tick",input.tick},{"time",input.time},{"timestep",input.timestep},{"images",json::array()},{"feedback",json::array()}};
  for(const auto& packet:input.observations) {
    ensure(bool(packet.image) && packet.observations.empty(),"External adapter accepts image-only camera packets");
    const auto& image=*packet.image;
    ensure(step.payload.size()+image.bgr.size()<=max_payload_bytes,"External image batch exceeds 64 MiB");
    step.header["images"].push_back({{"sequence",packet.sequence},{"sensor_id",packet.sensor_id},{"calibration_id",packet.calibration_id},
      {"capture_tick",packet.capture_tick},{"ready_tick",packet.ready_tick},{"capture_time",packet.capture_time},{"ready_time",packet.ready_time},
      {"width",image.width},{"height",image.height},{"stride",image.width*3},{"encoding","bgr8"},
      {"offset",step.payload.size()},{"bytes",image.bgr.size()}});
    step.payload.insert(step.payload.end(),image.bgr.begin(),image.bgr.end());
  }
  double sign=profile(config_.adapter).pitch_sign;
  for(const auto& f:input.feedback)step.header["feedback"].push_back({{"sequence",f.sequence},
    {"capture_tick",f.capture_tick},{"ready_tick",f.ready_tick},{"capture_time",f.capture_time},{"ready_time",f.ready_time},
    {"yaw",f.yaw},{"pitch",f.pitch*sign},{"yaw_velocity",f.yaw_velocity},{"pitch_velocity",f.pitch_velocity*sign}});
  auto deadline=deadline_after(milliseconds(config_.step_timeout));
  connection_.send(step,deadline);auto reply=connection_.receive(deadline);empty_payload(reply);
  keys(reply.header,{"protocol","type","session_id","sequence","tick","commands","telemetry"});
  same(reply.header,"type","commands");same(reply.header,"session_id",session_);
  same(reply.header,"sequence",sequence_);same(reply.header,"tick",input.tick);
  const auto& commands=reply.header.at("commands");
  ensure(commands.is_array() && commands.size()<=64,"External commands must be an array with at most 64 entries");
  std::vector<ControlRequest> result;
  for(const auto& command:commands){
    keys(command,{"control","yaw","pitch","fire","request_id"});
    bool control=command.at("control").get<bool>(),fire=command.value("fire",false);
    ensure(control || !fire,"Disabled external control cannot request a shot");
    double yaw=angle(command.value("yaw",json(0))),pitch=angle(command.value("pitch",json(0)));
    std::string id=command.value("request_id",std::string());
    ensure(id.size()<=128 && (!fire || !id.empty()),"External single-shot fire needs a nonempty request_id of at most 128 bytes");
    if(control){
      ensure(command.contains("yaw") && command.contains("pitch"),"External position control needs yaw and pitch");
      result.push_back({yaw,pitch*sign,fire,id});
    }
  }
  if(reply.header.contains("telemetry")){
    validate_telemetry(reply.header["telemetry"]);telemetry_=reply.header["telemetry"];
  }
  images_+=input.observations.size();feedback_+=input.feedback.size();commands_+=result.size();++sequence_;
  return result;
}
void ExternalAlgorithm::finish(Tick tick) {
  ensure(started_ && !finished_,"External adapter session is not active");
  auto deadline=deadline_after(milliseconds(config_.step_timeout));
  connection_.send({{{"protocol",wire_version},{"type","finish"},{"session_id",session_},{"tick",tick}},{}},deadline);
  auto reply=connection_.receive(deadline);empty_payload(reply);
  keys(reply.header,{"protocol","type","session_id","tick"});
  same(reply.header,"type","finished");same(reply.header,"session_id",session_);same(reply.header,"tick",tick);
  finished_=true;
  std::cout<<"[adapter] finished "<<sequence_<<" exchanges"<<std::endl;
}
json ExternalAlgorithm::statistics()const{return {{"profile",config_.adapter},{"peer_version",peer_version_},
  {"state",finished_?"finished":started_?"connected":"disconnected"},{"completed_steps",sequence_},
  {"images_sent",images_},{"feedback_sent",feedback_},{"commands_received",commands_},{"telemetry",telemetry_}};}
} // namespace rm_sim::adapters
