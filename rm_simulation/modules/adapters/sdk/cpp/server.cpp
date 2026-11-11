#include "rm_sim/adapters/server.hpp"
#include "rm_sim/adapters/profile.hpp"
#include <cmath>
#include <set>
#include <stdexcept>

namespace rm_sim::adapters {
using nlohmann::json;
namespace {
void require(bool ok,const char* text){if(!ok)throw std::invalid_argument(text);}
void keys(const json& object,std::initializer_list<const char*> allowed){
  require(object.is_object(),"Expected protocol object");std::set<std::string> names;
  for(auto s:allowed)names.insert(s);
  for(auto it=object.begin();it!=object.end();++it)require(names.count(it.key())!=0,"Unknown protocol field");
}
uint64_t integer(const json& j){require(j.is_number_unsigned(),"Expected unsigned protocol integer");return j.get<uint64_t>();}
void same(const json& j,const char* key,const json& value){require(j.contains(key) && j.at(key)==value,"Protocol session/sequence mismatch");}
void time_fields(const json& packet,uint64_t tick,double time){
  auto capture=integer(packet.at("capture_tick")),ready=integer(packet.at("ready_tick"));
  require(capture<=ready && ready<=tick,"Image/feedback delivered before ready tick");
  for(auto key:{"capture_time","ready_time"})require(packet.at(key).is_number(),"Invalid protocol time");
  double a=packet["capture_time"],b=packet["ready_time"];
  require(std::isfinite(a) && std::isfinite(b) && a>=0 && a<=b && b<=time+1e-9,"Invalid protocol time");
}
}
ProtocolServer::ProtocolServer(ServerOptions options):options_(std::move(options)){
  profile(options_.adapter);validate_endpoint(options_.host,options_.port);
  if(options_.accept_timeout_ms!=0)deadline_after(options_.accept_timeout_ms);
  deadline_after(options_.idle_timeout_ms);
  require(!options_.peer_version.empty() && options_.peer_version.size()<=128,"Invalid peer version");
  listener_=std::make_unique<Listener>(options_.host,options_.port);
}
void ProtocolServer::accept(){
  require(!active_,"Finish the current session before accepting another");
  if(!listener_)listener_=std::make_unique<Listener>(options_.host,options_.port);
  auto deadline=options_.accept_timeout_ms==0 ? Deadline::max():deadline_after(options_.accept_timeout_ms);
  connection_=listener_->accept(deadline);
  deadline=deadline_after(options_.idle_timeout_ms);
  auto message=connection_.receive(deadline);require(message.payload.empty(),"Hello must not contain pixels");
  auto& h=message.header;keys(h,{"protocol","type","session_id","adapter","units","configuration"});
  same(h,"type","hello");same(h,"adapter",options_.adapter);
  auto session=h.at("session_id").get<std::string>();require(!session.empty() && session.size()<=128,"Invalid session id");
  require(h.at("configuration").is_object() && h.at("units").is_object(),"Missing nominal configuration/units");
  same(h["units"],"pitch_up_sign",profile(options_.adapter).pitch_sign);
  connection_.send({{{"protocol",wire_version},{"type","hello_ack"},{"session_id",session},
    {"adapter",options_.adapter},{"peer_version",options_.peer_version}},{}},deadline);
  hello_=std::move(h);sequence_=last_tick_=0;pending_=nullptr;active_=true;
}
std::optional<Message> ProtocolServer::next(){
  require(active_ && pending_.is_null(),"Reply once before requesting the next step");
  auto deadline=deadline_after(options_.idle_timeout_ms);auto message=connection_.receive(deadline);
  const auto& h=message.header;same(h,"session_id",hello_["session_id"]);
  if(h.at("type")=="finish"){
    keys(h,{"protocol","type","session_id","tick"});
    auto tick=integer(h.at("tick"));require(sequence_==0 || tick>last_tick_,"Invalid session end tick");
    require(message.payload.empty(),"Finish must not contain pixels");
    connection_.send({{{"protocol",wire_version},{"type","finished"},{"session_id",hello_["session_id"]},{"tick",tick}},{}},deadline);
    active_=false;return std::nullopt;
  }
  keys(h,{"protocol","type","session_id","sequence","tick","time","timestep","images","feedback"});
  same(h,"type","step");require(integer(h.at("sequence"))==sequence_,"Unexpected step sequence");
  auto tick=integer(h.at("tick"));require(sequence_==0 || tick>last_tick_,"Simulation ticks must advance");
  require(h.at("time").is_number() && h.at("timestep").is_number(),"Invalid step clock");
  double time=h["time"],dt=h["timestep"];
  require(std::isfinite(time) && std::isfinite(dt) && dt>0 && std::abs(time-tick*dt)<1e-8,"Invalid step clock");
  require(h.at("images").is_array() && h.at("feedback").is_array(),"Invalid input channels");
  uint64_t expected_offset=0;
  for(const auto& image:h["images"]){
    keys(image,{"sequence","sensor_id","calibration_id","capture_tick","ready_tick","capture_time","ready_time",
      "width","height","stride","encoding","offset","bytes"});
    time_fields(image,tick,time);integer(image.at("sequence"));
    auto w=integer(image.at("width")),height=integer(image.at("height"));
    require(w>0 && w<=65536 && height>0 && height<=65536 && w*height<=16777216,"Invalid BGR dimensions");
    same(image,"encoding","bgr8");require(integer(image.at("stride"))==w*3,"Invalid BGR stride");
    auto bytes=integer(image.at("bytes"));
    require(integer(image.at("offset"))==expected_offset && bytes==w*height*3 && bytes<=max_payload_bytes-expected_offset,
      "Invalid image payload range");expected_offset+=bytes;
  }
  require(expected_offset==message.payload.size(),"Image payload length mismatch");
  for(const auto& feedback:h["feedback"]){
    keys(feedback,{"sequence","capture_tick","ready_tick","capture_time","ready_time","yaw","pitch","yaw_velocity","pitch_velocity"});
    time_fields(feedback,tick,time);integer(feedback.at("sequence"));
    for(auto key:{"yaw","pitch","yaw_velocity","pitch_velocity"})
      require(feedback.at(key).is_number() && std::isfinite(feedback.at(key).get<double>()),"Invalid feedback value");
  }
  pending_=h;last_tick_=tick;return message;
}
void ProtocolServer::reply(const std::vector<Command>& commands,const json& telemetry){
  require(active_ && !pending_.is_null(),"No pending step to reply to");
  require(commands.size()<=64,"At most 64 commands per step");json values=json::array();
  for(const auto& c:commands){
    require(std::isfinite(c.yaw) && std::isfinite(c.pitch) && std::abs(c.yaw)<=1e6 && std::abs(c.pitch)<=1e6,"Invalid command angle");
    require((c.control || !c.fire) && (!c.fire || !c.request_id.empty()) && c.request_id.size()<=128,"Invalid single-shot request");
    values.push_back({{"control",c.control},{"yaw",c.yaw},{"pitch",c.pitch},{"fire",c.fire},{"request_id",c.request_id}});
  }
  validate_telemetry(telemetry);
  json header={{"protocol",wire_version},{"type","commands"},{"session_id",hello_["session_id"]},
    {"sequence",pending_["sequence"]},{"tick",pending_["tick"]},{"commands",values}};
  if(!telemetry.empty())header["telemetry"]=telemetry;
  connection_.send({header,{}},deadline_after(options_.idle_timeout_ms));
  pending_=nullptr;++sequence_;
}
} // namespace rm_sim::adapters
