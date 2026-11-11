#include "rm_sim/runner.hpp"
#include "rm_sim/adapters/external.hpp"
#include "rm_sim/adapters/live.hpp"
#include "live_output.hpp"
#include "rm_sim/scenario.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <csignal>
#include "rm_sim/image_io.hpp"

using namespace rm_sim;
using namespace rm_sim::scenario;
namespace {
volatile std::sig_atomic_t live_stop=0;
Pose pose(const json& j,Pose fallback) {
  keys(j,{"translation","rotation"});
  if (j.contains("translation")) fallback.translation=vec(j["translation"]);
  if (j.contains("rotation")) {
    const auto& r=j["rotation"];
    ensure(r.is_array() && r.size()==3,"Rotation needs three column vectors");
    fallback.rotation={vec(r[0]),vec(r[1]),vec(r[2])};
  }
  fallback.validate(); return fallback;
}
json encode(const Pose& p) {
  return {{"translation",js(p.translation)},
    {"rotation",{js(p.rotation.x),js(p.rotation.y),js(p.rotation.z)}}};
}
json encode(const CameraCalibration& k) {
  return {{"id",k.id},{"width",k.width},{"height",k.height},{"fx",k.fx},{"fy",k.fy},{"cx",k.cx},{"cy",k.cy}};
}
CameraCalibration calibration(const json& j,CameraCalibration c) {
  keys(j,{"id","width","height","fx","fy","cx","cy"});
  for (auto key:{"width","height"}) if (j.contains(key)) ensure(j[key].is_number_integer(),"Image dimensions must be integers");
#define FIELD(name) c.name=j.value(#name,c.name)
  FIELD(id); FIELD(width); FIELD(height); FIELD(fx); FIELD(fy); FIELD(cx); FIELD(cy);
#undef FIELD
  c.validate(); return c;
}
uint64_t read_seed(const json& j,const char* key,uint64_t fallback) {
  if (!j.contains(key)) return fallback;
  ensure(j[key].is_number_unsigned(),"Seed must be an unsigned integer");
  return j[key].get<uint64_t>();
}
SensorConfig sensor_config(const json& j,SensorConfig c) {
  keys(j,{"enabled","surface_occlusion","id","calibration","frequency","latency","pixel_sigma",
    "dropout","false_positive_probability","seed","appearances","output","image"});
#define FIELD(name) c.name=j.value(#name,c.name)
  FIELD(enabled); FIELD(surface_occlusion); FIELD(id); FIELD(frequency); FIELD(latency); FIELD(output);
  FIELD(pixel_sigma); FIELD(dropout); FIELD(false_positive_probability);
#undef FIELD
  if (j.contains("image")) {
    const auto& image=j["image"];
    keys(image,{"color","renderer","light_length","light_width","small_light_spacing","big_light_spacing","noise_sigma"});
#define IMAGE_FIELD(name) c.image.name=image.value(#name,c.image.name)
    IMAGE_FIELD(color); IMAGE_FIELD(light_length); IMAGE_FIELD(light_width);
    IMAGE_FIELD(small_light_spacing); IMAGE_FIELD(big_light_spacing); IMAGE_FIELD(noise_sigma); IMAGE_FIELD(renderer);
#undef IMAGE_FIELD
  }
  c.seed=read_seed(j,"seed",c.seed);
  if (j.contains("calibration")) c.calibration=calibration(j["calibration"],c.calibration);
  if (j.contains("appearances")) {
    ensure(j["appearances"].is_object(),"Appearances must be an object keyed by target id");
    for (auto it=j["appearances"].begin();it!=j["appearances"].end();++it) {
      keys(it.value(),{"model_id","measured_class","points"});
      Appearance a;
      a.model_id=it.value().at("model_id"); a.measured_class=it.value().at("measured_class");
      ensure(it.value().at("points").is_array(),"Appearance points must be an array");
      for (const auto& point:it.value()["points"]) a.points.push_back(vec(point));
      c.appearances.emplace(it.key(),std::move(a));
    }
  }
  return c;
}
json encode(const SensorConfig& c) {
  json appearances=json::object();
  for (const auto& item:c.appearances) {
    json points=json::array(); for (auto p:item.second.points) points.push_back(js(p));
    appearances[item.first]={{"model_id",item.second.model_id},{"measured_class",item.second.measured_class},{"points",points}};
  }
  return {{"enabled",c.enabled},{"surface_occlusion",c.surface_occlusion},{"id",c.id},{"output",c.output},
    {"image",{{"color",c.image.color},{"light_length",c.image.light_length},{"light_width",c.image.light_width},
      {"small_light_spacing",c.image.small_light_spacing},{"big_light_spacing",c.image.big_light_spacing},
      {"noise_sigma",c.image.noise_sigma},{"renderer",c.image.renderer}}},
    {"calibration",encode(c.calibration)},{"frequency",c.frequency},{"latency",c.latency},
    {"pixel_sigma",c.pixel_sigma},{"dropout",c.dropout},{"false_positive_probability",c.false_positive_probability},
    {"seed",c.seed},{"appearances",appearances}};
}
ActuatorConfig actuator_config(const json& j,ActuatorConfig c) {
  keys(j,{"ideal","wrap_yaw","inherit_muzzle_velocity","align_on_start","base_pose","yaw_offset","pitch_offset",
    "camera_mount","muzzle_mount","initial_yaw","initial_pitch","yaw_min","yaw_max","pitch_min","pitch_max",
    "max_speed","max_acceleration","response_time","command_latency","fire_latency","fire_interval",
    "muzzle_speed","muzzle_speed_sigma","shot_lifetime","feedback_enabled","feedback_frequency",
    "feedback_latency","encoder_sigma","seed","feedback_seed"});
#define FIELD(name) c.name=j.value(#name,c.name)
  FIELD(ideal); FIELD(wrap_yaw); FIELD(inherit_muzzle_velocity); FIELD(align_on_start); FIELD(initial_yaw); FIELD(initial_pitch);
  FIELD(yaw_min); FIELD(yaw_max); FIELD(pitch_min); FIELD(pitch_max); FIELD(max_speed); FIELD(max_acceleration);
  FIELD(response_time); FIELD(command_latency); FIELD(fire_latency); FIELD(fire_interval); FIELD(muzzle_speed);
  FIELD(muzzle_speed_sigma); FIELD(shot_lifetime); FIELD(feedback_enabled); FIELD(feedback_frequency);
  FIELD(feedback_latency); FIELD(encoder_sigma);
#undef FIELD
  c.seed=read_seed(j,"seed",c.seed); c.feedback_seed=read_seed(j,"feedback_seed",c.feedback_seed);
  if (j.contains("base_pose")) c.base_pose=pose(j["base_pose"],c.base_pose);
  if (j.contains("camera_mount")) c.camera_mount=pose(j["camera_mount"],c.camera_mount);
  if (j.contains("muzzle_mount")) c.muzzle_mount=pose(j["muzzle_mount"],c.muzzle_mount);
  if (j.contains("yaw_offset")) c.yaw_offset=vec(j["yaw_offset"]);
  if (j.contains("pitch_offset")) c.pitch_offset=vec(j["pitch_offset"]);
  return c;
}
json encode(const ActuatorConfig& c) {
  json j;
#define FIELD(name) j[#name]=c.name
  FIELD(ideal); FIELD(wrap_yaw); FIELD(inherit_muzzle_velocity); FIELD(align_on_start); FIELD(initial_yaw); FIELD(initial_pitch);
  FIELD(yaw_min); FIELD(yaw_max); FIELD(pitch_min); FIELD(pitch_max); FIELD(max_speed); FIELD(max_acceleration);
  FIELD(response_time); FIELD(command_latency); FIELD(fire_latency); FIELD(fire_interval); FIELD(muzzle_speed);
  FIELD(muzzle_speed_sigma); FIELD(shot_lifetime); FIELD(feedback_enabled); FIELD(feedback_frequency);
  FIELD(feedback_latency); FIELD(encoder_sigma); FIELD(seed); FIELD(feedback_seed);
#undef FIELD
  j["base_pose"]=encode(c.base_pose); j["camera_mount"]=encode(c.camera_mount); j["muzzle_mount"]=encode(c.muzzle_mount);
  j["yaw_offset"]=js(c.yaw_offset); j["pitch_offset"]=js(c.pitch_offset);
  return j;
}
json encode(const SensorPacket& p) {
  json observations=json::array();
  for (const auto& o:p.observations) {
    json pixels=json::array(); for (auto xy:o.pixels) pixels.push_back({xy.x,xy.y});
    observations.push_back({{"model_id",o.model_id},{"measured_class",o.measured_class},
      {"pixels",pixels},{"covariance",o.covariance}});
  }
  return {{"sequence",p.sequence},{"capture_tick",p.capture_tick},{"ready_tick",p.ready_tick},
    {"capture_time",p.capture_time},{"ready_time",p.ready_time},{"sensor_id",p.sensor_id},
    {"calibration_id",p.calibration_id},{"observations",observations}};
}
json encode(const ActuatorTruth& a) {
  return {{"tick",a.tick},{"yaw",a.yaw},{"pitch",a.pitch},{"yaw_velocity",a.yaw_velocity},
    {"pitch_velocity",a.pitch_velocity},{"camera_pose",encode(a.camera_pose)},{"muzzle_pose",encode(a.muzzle_pose)},
    {"muzzle_velocity",js(a.muzzle_velocity)},{"next_launch_tick",a.next_launch_tick}};
}
class NoAlgorithm : public Algorithm {
 public:
  std::string version() const override { return "none/1"; }
  std::vector<ControlRequest> update(const AlgorithmInput&) override { return {}; }
};
} // namespace
int main(int argc,char** argv) {
  try {
    if (argc==2 && std::string(argv[1])=="--help") {
      std::cout<<"Usage: rm_sim_loop closed-loop.json trace.json | --validate closed-loop.json | --live closed-loop.json [--live-for=seconds]\n"; return 0;
    }
    const bool live=argc>=2 && std::string(argv[1])=="--live";
    ensure(argc==3 || (live && argc==4),"Usage: rm_sim_loop configuration.json trace.json | --live configuration.json");
    double live_limit=0;
    if(live && argc==4){
      std::string option=argv[3];ensure(option.rfind("--live-for=",0)==0,"Unknown live option");
      auto number=option.substr(11);size_t parsed=0;live_limit=std::stod(number,&parsed);
      ensure(parsed==number.size() && std::isfinite(live_limit) && live_limit>0 && live_limit<=600,"Invalid live test limit");
    }
    const bool validate_only=std::string(argv[1])=="--validate";
    std::ifstream input(argv[validate_only || live ? 2 : 1]); ensure(bool(input),"Cannot open closed-loop configuration");
    const json source=json::parse(input);
    keys(source,{"closed_loop_version","scene","sensor","actuator","runner","algorithm","commands"});
    ensure(source.at("closed_loop_version")==1,"Unsupported closed-loop configuration version");
    const auto& scene=source.at("scene");
    keys(scene,{"schema_version","name","duration","sample_period","world","targets","shots","events"});
    ensure(scene.at("schema_version")==1,"Unsupported scene version");
    RunnerConfig cfg;
    if (scene.contains("world")) {
      const auto& w=scene["world"];
      keys(w,{"timestep","gravity","ground_z","minimum_normal_speed","detection_interval"});
      cfg.world.timestep=w.value("timestep",cfg.world.timestep);
      cfg.world.ground_z=w.value("ground_z",cfg.world.ground_z);
      cfg.world.minimum_normal_speed=w.value("minimum_normal_speed",cfg.world.minimum_normal_speed);
      cfg.world.detection_interval=w.value("detection_interval",cfg.world.detection_interval);
      if (w.contains("gravity")) cfg.world.gravity=vec(w["gravity"]);
    }
    World validate_world(cfg.world);
    const Tick duration=ticks(scene.at("duration"),cfg.world.timestep);
    const auto stride=ticks(scene.value("sample_period",.02),cfg.world.timestep);
    ensure(stride>0,"Sample period must be positive");
    cfg.log_frequency=1/(stride*cfg.world.timestep);
    if (source.contains("sensor")) cfg.sensor=sensor_config(source["sensor"],cfg.sensor);
    // Old explicitly selected PixelServo configurations retain their feature protocol.
    if ((!source.contains("sensor") || !source["sensor"].contains("output")) &&
        source.contains("algorithm") && source["algorithm"].value("kind",std::string())=="pixel_servo")
      cfg.sensor.output="features";
    cfg.actuator.align_on_start=live;
    if (source.contains("actuator")) cfg.actuator=actuator_config(source["actuator"],cfg.actuator);
    if (source.contains("runner")) {
      const auto& r=source["runner"]; keys(r,{"algorithm_frequency","log_frequency"});
      cfg.algorithm_frequency=r.value("algorithm_frequency",cfg.algorithm_frequency);
      cfg.log_frequency=r.value("log_frequency",cfg.log_frequency);
    }
    std::vector<TargetConfig> targets;
    ensure(scene.at("targets").is_array() && !scene["targets"].empty() && scene["targets"].size()<=64,
      "Need 1..64 targets");
    for (const auto& t:scene["targets"]) targets.push_back(target(t));
    std::vector<SceneEvent> events;
    auto read_array=[](const json& j,const char* key) {
      auto a=j.value(key,json::array()); ensure(a.is_array(),"Expected array"); return a;
    };
    for (const auto& e:read_array(scene,"events")) {
      keys(e,{"time","type","target_id"});
      events.push_back({ticks(e.at("time"),cfg.world.timestep),e.at("type"),e.at("target_id")});
    }
    std::vector<ScheduledShot> shots;
    for (const auto& s:read_array(scene,"shots")) {
      keys(s,{"time","id","origin","velocity","lifetime"});
      shots.push_back({ticks(s.at("time"),cfg.world.timestep),
        {s.at("id"),vec(s.at("origin")),vec(s.at("velocity")),s.value("lifetime",3.)}});
    }
    Runner runner(cfg,targets,events,shots);
    cfg=runner.config();
    for (const auto& c:read_array(source,"commands")) {
      keys(c,{"time","yaw","pitch","fire","request_id"});
      Tick due=ticks(c.at("time"),cfg.world.timestep);
      ensure(due<duration,"Prequeued command must precede run end");
      runner.schedule({c.at("yaw"),c.at("pitch"),c.value("fire",false),c.value("request_id",std::string())},due);
    }
    auto a=source.value("algorithm",json::object());
    keys(a,{"kind","gain","fire_tolerance","fire_interval","max_observation_age","fire",
      "adapter","host","port","connect_timeout","step_timeout"});
    std::string kind=a.value("kind",cfg.sensor.output=="image" ? std::string("none"):std::string("pixel_servo"));
    std::unique_ptr<Algorithm> algorithm;
    adapters::ExternalAlgorithm* external=nullptr;
    adapters::LiveExternal* live_external=nullptr;
    json resolved_algorithm;
    PixelServoConfig servo; servo.calibration=cfg.sensor.calibration;
    servo.gain=a.value("gain",servo.gain); servo.fire_tolerance=a.value("fire_tolerance",servo.fire_tolerance);
    servo.fire_interval=a.value("fire_interval",servo.fire_interval);
    servo.max_observation_age=a.value("max_observation_age",servo.max_observation_age); servo.fire=a.value("fire",servo.fire);
    if (kind=="pixel_servo") {
      keys(a,{"kind","gain","fire_tolerance","fire_interval","max_observation_age","fire"});
      ensure(cfg.sensor.output=="features","PixelServo requires sensor.output=features; image mode uses none or external");
      algorithm=std::make_unique<PixelServo>(servo);
    }
    else if(kind=="external") {
      auto config=adapters::parse_config(a);config.validate(cfg);
      if(live){
        auto instance=std::make_unique<adapters::LiveExternal>(config,cfg);
        live_external=instance.get();algorithm=std::move(instance);
      }else{
        auto instance=std::make_unique<adapters::ExternalAlgorithm>(config,cfg,duration);
        external=instance.get();algorithm=std::move(instance);
      }
      resolved_algorithm=adapters::encode_config(config);
    }
    else { ensure(kind=="none","Unknown algorithm kind"); keys(a,{"kind"}); algorithm=std::make_unique<NoAlgorithm>(); }
    if(kind!="external") resolved_algorithm={{"kind",kind},{"gain",servo.gain},{"fire",servo.fire},
      {"fire_tolerance",servo.fire_tolerance},{"fire_interval",servo.fire_interval},{"max_observation_age",servo.max_observation_age}};
    runner.validate_duration(duration);
    if (validate_only) { std::cout<<"Configuration valid\n"; return 0; }
    if(live){
      // stdout is reserved for framed live pixels/state; diagnostics go to stderr.
      std::cout.rdbuf(std::cerr.rdbuf());
      std::signal(SIGINT,[](int){live_stop=1;});std::signal(SIGTERM,[](int){live_stop=1;});
      std::signal(SIGPIPE,SIG_IGN);
      if(live_external)live_external->start();
      LiveOutput output;uint64_t sequence=0;
      auto tick=runner.run_live(*algorithm,[]{return live_stop!=0;},[&](const LiveUpdate& update){
        json header={{"protocol","rm-live/1"},{"type","frame"},{"sequence",sequence++},
          {"name",scene.value("name",std::string("实时仿真"))},{"frame",frame(update.frame.world)},
          {"actuator",encode(update.frame.actuator)},{"wall_time",update.wall_time},
          {"statistics",{{"shots",update.shots},{"commands",update.commands},{"camera_frames",update.camera_frames},
            {"camera_frequency",cfg.sensor.frequency},
            {"dropped_camera_jobs",update.dropped_camera_jobs},{"contacts",update.contacts},{"effective_hits",update.effective_hits}}},
          {"hits",json::array()}};
        for(const auto& h:update.hits)header["hits"].push_back({{"time",h.time},{"shot_id",h.shot_id},
          {"target_id",h.target_id},{"plate_id",h.plate_id},{"point",js(h.point)},{"plate_point",h.plate_point ? js(*h.plate_point) : json(nullptr)},
          {"normal_speed",h.normal_speed},{"effective",h.effective},{"reason",h.reason},{"ring",h.ring}});
        std::shared_ptr<const Image> image;
        if(update.camera){
          const auto& p=*update.camera;image=p.image;
          header["camera"]={{"sequence",p.sequence},{"capture_time",p.capture_time},{"ready_time",p.ready_time},
            {"renderer",image->renderer},{"device",image->device},
            {"width",image->width},{"height",image->height},{"stride",image->width*3},{"encoding","bgr8"}};
        }
        if(live_external)header["adapter"]=live_external->statistics();
        output.publish(std::move(header),std::move(image));
      },live_limit);
      if(live_external)live_external->finish(tick);
      output.close();return 0;
    }
    if(external)external->start();
    std::map<uint64_t,json> image_records;
    ImageArchive images(argv[2]);
    runner.image_sink=[&](const SensorPacket& packet) {
      image_records.emplace(packet.sequence,json{{"file",images.write(*packet.image)},
        {"width",packet.image->width},{"height",packet.image->height},{"encoding","bgr8"},
        {"storage","png"}});
    };
    const auto& log=runner.run(duration,*algorithm);
    if(external)external->finish(log.end_tick);
    json output={{"schema_version",1},{"closed_loop_version",1},{"engine","rm_sim_loop/1"},
      {"scenario",scene},{"configuration",source},
      {"versions",{{"world","rm_sim_world/1"},{"sensor",cfg.sensor.output=="image" ? "synthetic-bgr-camera/1":"geometric-features/1"},{"actuator","position-actuator/1"},
        {"runner","tick-runner/1"},{"algorithm",algorithm->version()}}},
      {"resolved",{{"sensor",encode(cfg.sensor)},{"actuator",encode(cfg.actuator)},
        {"world",{{"timestep",cfg.world.timestep},{"gravity",js(cfg.world.gravity)},
          {"ground_z",cfg.world.ground_z},{"minimum_normal_speed",cfg.world.minimum_normal_speed},
          {"detection_interval",cfg.world.detection_interval}}},
        {"algorithm_frequency",cfg.algorithm_frequency},{"log_frequency",cfg.log_frequency},
        {"algorithm",resolved_algorithm}}},
      {"timing",{{"quantization","ceil to physics tick"},{"minimum_algorithm_command_delay_ticks",1},
        {"end_tick",log.end_tick},{"end_is_exclusive",true}}},
      {"model","synthetic pinhole BGR images or legacy features; simplified opaque geometry; no lens distortion or exposure integration; point projectiles"},
      {"frames",json::array()},{"hits",json::array()},
      {"algorithm_channels",{{"observations",json::array()},{"feedback",json::array()}}},
      {"commands",json::array()},{"launches",json::array()},
      {"evaluation",{{"annotations",json::array()},{"actuator_states",json::array()}}}};
    for (const auto& f:log.frames) {
      output["frames"].push_back(frame(f.world));
      output["evaluation"]["actuator_states"].push_back(encode(f.actuator));
    }
    for (const auto& p:log.observations) {
      auto packet=encode(p);
      auto image=image_records.find(p.sequence);
      if (image!=image_records.end()) packet["image"]=image->second;
      output["algorithm_channels"]["observations"].push_back(std::move(packet));
    }
    for (const auto& f:log.feedback)
      output["algorithm_channels"]["feedback"].push_back({{"sequence",f.sequence},{"capture_tick",f.capture_tick},
        {"ready_tick",f.ready_tick},{"capture_time",f.capture_time},{"ready_time",f.ready_time},
        {"yaw",f.yaw},{"pitch",f.pitch},{"yaw_velocity",f.yaw_velocity},{"pitch_velocity",f.pitch_velocity}});
    for (const auto& a:log.annotations) {
      json features=json::array();
      for (const auto& f:a.features) {
        json pixels=json::array(); for (auto xy:f.ideal_pixels) pixels.push_back({xy.x,xy.y});
        features.push_back({{"target_id",f.target_id},{"plate_id",f.plate_id},{"observation_index",f.observation_index},
          {"ideal_pixels",pixels},{"visible",f.visible},{"reason",f.reason}});
      }
      output["evaluation"]["annotations"].push_back({{"sequence",a.sequence},{"capture_tick",a.capture_tick},
        {"camera_pose",encode(a.camera_pose)},{"features",features}});
    }
    for (const auto& c:log.commands) output["commands"].push_back({{"sequence",c.sequence},{"issued_tick",c.issued_tick},
      {"due_tick",c.due_tick},{"yaw",c.request.yaw},{"pitch",c.request.pitch},{"fire",c.request.fire},{"request_id",c.request.request_id}});
    for (const auto& l:log.launches) output["launches"].push_back({{"tick",l.tick},{"command_sequence",l.command_sequence},
      {"request_id",l.request_id},{"shot_id",l.shot.id},{"origin",js(l.shot.origin)},
      {"velocity",js(l.shot.velocity)},{"lifetime",l.shot.lifetime}});
    int accepted=0;
    for (const auto& h:log.hits) {
      accepted+=h.effective;
      output["hits"].push_back({{"time",h.time},{"shot_id",h.shot_id},{"target_id",h.target_id},{"plate_id",h.plate_id},
        {"point",js(h.point)},{"plate_point",h.plate_point ? js(*h.plate_point) : json(nullptr)},{"normal_speed",h.normal_speed},{"effective",h.effective},{"reason",h.reason},{"ring",h.ring}});
    }
    output["summary"]={{"shots",log.launches.size()},{"contacts",log.hits.size()},{"effective_hits",accepted},
      {"sensor_packets",log.observations.size()},{"image_frames",image_records.size()},{"commands",log.commands.size()},
      {"pending_observations",log.pending_observations},{"pending_feedback",log.pending_feedback},
      {"pending_commands",log.pending_commands},{"pending_launches",log.pending_launches}};
    if(external)output["adapter"]=external->statistics();
    std::ofstream out(argv[2]); ensure(bool(out),"Cannot open output trace");
    out<<output.dump(); ensure(bool(out),"Cannot write output trace");
    std::cout<<output["summary"].dump()<<'\n';
  } catch (const std::exception& e) { std::cerr<<"[rm_sim_loop] "<<e.what()<<'\n'; return 1; }
}
