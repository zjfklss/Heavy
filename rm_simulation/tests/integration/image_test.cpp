#include "rm_sim/runner.hpp"
#include "rm_sim/armor_markings.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace rm_sim;
namespace {
int checks=0;
std::string renderer="auto";
void check(bool ok,const char* message) { ++checks;if(!ok)throw std::runtime_error(message); }
void near(double a,double b,double tolerance=.7) { if(std::abs(a-b)>=tolerance)throw std::runtime_error("Pinhole mismatch actual="+std::to_string(a)+" expected="+std::to_string(b));++checks; }
void rejects(const std::function<void()>& fn) { bool failed=false;try{fn();}catch(const std::invalid_argument&){failed=true;}check(failed,"Invalid camera input accepted"); }
SensorConfig config() {
  SensorConfig c;c.calibration.width=320;c.calibration.height=240;
  c.calibration.fx=c.calibration.fy=400;c.calibration.cx=160;c.calibration.cy=120;c.image.renderer=renderer;return c;
}
Snapshot panel(double x=2,double y=0,double z=.3) {
  Snapshot s;TargetState t{};t.id="test";t.kind=TargetKind::car;
  Surface p;p.target_id=t.id;p.center={x,y,z};p.normal={-1,0,0};p.right={0,-1,0};p.up={0,0,1};
  p.width=.135;p.height=.125;t.surfaces.push_back(p);s.targets.push_back(t);return s;
}
struct Lights { double x=0,y=0,left=0,right=0;int count=0,lc=0,rc=0; };
Lights lights(const Image& image,double split=160,bool red=false) {
  Lights out;
  for(int y=0;y<image.height;++y)for(int x=0;x<image.width;++x){
    auto p=&image.bgr[(size_t(y)*image.width+x)*3];
    if(p[red?2:0]>100 && p[red?2:0]>p[red?0:2]*2 && p[red?2:0]>p[1]*2){
      ++out.count;out.x+=x+.5;out.y+=y+.5;
      if(x+.5<split){out.left+=x+.5;++out.lc;}else{out.right+=x+.5;++out.rc;}
    }
  }
  if(out.count){out.x/=out.count;out.y/=out.count;}
  if(out.lc)out.left/=out.lc;if(out.rc)out.right/=out.rc;return out;
}
void projection_and_appearance() {
  auto c=config();Actuator a({},.001);Sensor sensor(c,.001);auto truth=panel();
  auto sample=sensor.capture(truth,a.truth().camera_pose,0);
  check(bool(sample.packet.image),"Camera did not return pixels");
  check(sample.packet.observations.empty(),"Image mode leaked geometric observations");
  auto& image=*sample.packet.image;check(image.width==320 && image.height==240 && image.bgr.size()==320*240*3,"Wrong BGR image shape");
  if(renderer!="auto")check(image.renderer==renderer,"Requested camera backend was not used");
  auto l=lights(image);check(l.count>20,"Blue lights absent");
  near(l.left,160-400*.135/4);near(l.right,160+400*.135/4);near(l.y,120);
  auto shifted=panel(2,.2,.4);Sensor shifted_sensor(c,.001);
  auto offset=lights(*shifted_sensor.capture(shifted,a.truth().camera_pose,0).packet.image);
  near(offset.x,160-400*.2/2);near(offset.y,120-400*.1/2);
  c.calibration.fx=800;Sensor zoom(c,.001);
  auto enlarged=lights(*zoom.capture(truth,a.truth().camera_pose,0).packet.image);
  near(enlarged.right-enlarged.left,800*.135/2,1);
  c=config();c.image.color="red";Sensor red(c,.001);
  auto red_image=red.capture(truth,a.truth().camera_pose,0).packet.image;
  check(lights(*red_image).count==0 && lights(*red_image,160,true).count>20,"BGR color ordering incorrect");
  c=config();c.image.small_light_spacing=.10;Sensor compact(c,.001);
  auto compact_lights=lights(*compact.capture(truth,a.truth().camera_pose,0).packet.image);
  near(compact_lights.right-compact_lights.left,400*.10/2,1);
  c=config();c.image.noise_sigma=2;Sensor n1(c,.001),n2(c,.001);
  auto noisy=n1.capture(truth,a.truth().camera_pose,0).packet.image;
  check(noisy->bgr==n2.capture(truth,a.truth().camera_pose,0).packet.image->bgr,"Seeded image noise is not reproducible");
  check(noisy->bgr!=image.bgr,"Image noise absent");
  c.seed++;Sensor n3(c,.001);check(noisy->bgr!=n3.capture(truth,a.truth().camera_pose,0).packet.image->bgr,"Image seed ignored");
}
void visibility() {
  auto c=config();Actuator a({},.001);Sensor s(c,.001);
  auto truth=panel();truth.targets[0].surfaces[0].normal={1,0,0};
  check(lights(*s.capture(truth,a.truth().camera_pose,0).packet.image).count==0,"Back-face lights visible");
  truth=panel(-2);check(lights(*s.capture(truth,a.truth().camera_pose,0).packet.image).count==0,"Behind-camera lights visible");
  truth=panel(2,10);check(lights(*s.capture(truth,a.truth().camera_pose,0).packet.image).count==0,"Offscreen lights visible");
  truth=panel();auto blocker=panel(1).targets[0];blocker.id="blocker";
  blocker.surfaces[0].normal={1,0,0};blocker.surfaces[0].width=.5;blocker.surfaces[0].height=.4;
  truth.targets.push_back(blocker);
  auto blocked=s.capture(truth,a.truth().camera_pose,0).packet.image;
  check(lights(*blocked).count==0,"Opaque foreground did not occlude lights");
  std::reverse(truth.targets.begin(),truth.targets.end());
  check(blocked->bgr==s.capture(truth,a.truth().camera_pose,0).packet.image->bgr,"Occlusion depends on target order");
  ActuatorConfig rotated;rotated.initial_yaw=.1;Actuator yaw(rotated,.001);
  auto moved=lights(*s.capture(panel(),yaw.truth().camera_pose,0).packet.image,160+400*std::tan(.1));
  near((moved.left+moved.right)/2,160+400*std::tan(.1),.8);
  // A surface straddling the near plane must clip safely rather than disappear or divide by zero.
  auto close=panel(.015);
  close.targets[0].surfaces[0].right={std::sqrt(.5),-std::sqrt(.5),0};
  close.targets[0].surfaces[0].normal={-std::sqrt(.5),-std::sqrt(.5),0};
  auto close_image=s.capture(close,a.truth().camera_pose,0).packet.image;
  check(close_image->bgr.size()==320*240*3,"Near-plane clipping damaged output");
}
void armor_identities() {
  auto c=config();Actuator a({},.001);Sensor sensor(c,.001);
  std::vector<std::vector<uint8_t>> images;
  for(const auto* id:{"1","2","3","4","sentry","outpost"}) {
    auto truth=panel();truth.targets[0].armor_marking=id;
    if(std::string(id)=="outpost")truth.targets[0].kind=TargetKind::outpost;
    auto result=sensor.capture(truth,a.truth().camera_pose,0);
    check(result.packet.observations.empty(),"Marking identity leaked into algorithm observations");
    const auto& pixels=result.packet.image->bgr;
    for(const auto& previous:images)check(previous!=pixels,"Different armor identities have identical pixels");
    images.push_back(pixels);
    auto light=lights(*result.packet.image);
    near(light.left,160-400*.135/4);near(light.right,160+400*.135/4);
    check(armor_marking(truth.targets[0]).id==id,"UI/camera resolved a different marking");
  }
  World world;TargetConfig three;three.id="three";
  TargetConfig four=three;four.id="four";four.armor_marking="4";four.position.y=1;
  world.add_target(three);world.add_target(four);
  auto state=world.snapshot();
  check(state.targets[0].armor_marking=="3" && state.targets[1].armor_marking=="4",
    "Different vehicles lost their individual markings");
  rejects([]{TargetConfig bad;bad.id="invalid";bad.armor_marking="5";bad.validate();});
  rejects([]{armor_marking("G");});
}
struct Receiver:Algorithm {
  std::vector<SensorPacket> packets;
  std::string version()const override{return "image-test/1";}
  std::vector<ControlRequest> update(const AlgorithmInput& in)override {
    for(auto p:in.observations){check(p.ready_tick<=in.tick,"Future image delivered");packets.push_back(std::move(p));}return {};
  }
};
void floating_plates_and_initial_view() {
  for(bool outpost:{false,true}) {
    TargetConfig t;t.id="floating";
    if(outpost)t=TargetConfig::outpost(t.id,{4,0,0});
    World world;world.add_target(t);
    auto truth=world.snapshot();Actuator a({},.001);Sensor s(config(),.001);
    auto original=s.capture(truth,a.truth().camera_pose,0).packet.image;
    truth.targets[0].position={2,0,.3};truth.targets[0].rotation.position=.7;
    auto changed=s.capture(truth,a.truth().camera_pose,0).packet.image;
    check(original->bgr==changed->bgr,"Decorative target body remains in camera image");
  }
  for(Vec3 position:std::vector<Vec3>{{4,2,0},{-4,2,0},{2,-4,.2}}) {
    RunnerConfig c;c.sensor=config();c.actuator.align_on_start=true;
    c.actuator.base_pose.rotation=compose({yaw_rotation(.35),{}},{pitch_rotation(.1),{}}).rotation;
    c.actuator.yaw_offset={.02,-.03,.04};c.actuator.pitch_offset={.04,.02,0};
    c.actuator.camera_mount.translation={.00352,0,.0975};
    auto t=TargetConfig::outpost("outpost",position);
    Receiver receiver;Runner runner(c,{t});auto& log=runner.run(3,receiver);
    const auto& frame=log.frames.front();double error=1e9;
    for(const auto& plate:frame.world.targets.front().surfaces) {
      auto local=frame.actuator.camera_pose.inverse(plate.center);
      if(local.z>0 && dot(plate.normal,frame.actuator.camera_pose.translation-plate.center)>0)
        error=std::min(error,std::hypot(local.x,local.y)/local.z);
    }
    check(error<1e-7,"Startup camera misses the mounted, elevated outpost armor");
    check(lights(*receiver.packets.front().image).count>0,"Aligned startup image has no armor lights");
    near(frame.actuator.yaw,runner.config().actuator.initial_yaw,1e-12);
    near(log.feedback.front().pitch,frame.actuator.pitch,1e-12);
    // Startup alignment must not continue fighting commands or moving target state.
    Receiver controlled;Runner commanded(c,{t});
    double goal=wrap_angle(commanded.config().actuator.initial_yaw+.2);
    commanded.schedule({goal,0,false,{}},1);
    auto& command_log=commanded.run(3,controlled);
    near(command_log.frames.back().actuator.yaw,goal,1e-12);
    near(command_log.frames.back().actuator.pitch,0,1e-12);
  }
  RunnerConfig c;c.sensor=config();c.actuator.initial_yaw=.12;c.actuator.initial_pitch=-.02;
  Receiver manual;Runner runner(c,{TargetConfig::outpost("manual",{4,0,0})});
  auto& log=runner.run(1,manual);
  near(log.frames.front().actuator.yaw,.12,1e-12);
  near(log.frames.front().actuator.pitch,-.02,1e-12);
  c.actuator.align_on_start=true;c.actuator.wrap_yaw=false;
  c.actuator.yaw_min=-.1;c.actuator.yaw_max=.2;c.actuator.initial_yaw=0;
  c.actuator.pitch_min=-.1;c.actuator.pitch_max=.15;
  Receiver bounded;Runner limited(c,{TargetConfig::outpost("limited",{4,4,0})});
  auto& limited_log=limited.run(1,bounded);
  near(limited_log.frames.front().actuator.yaw,.2,1e-8);
  near(limited_log.frames.front().actuator.pitch,.15,1e-8);
}
void timing_and_ownership() {
  RunnerConfig c;c.sensor=config();c.sensor.frequency=100;c.sensor.latency=.0101;c.algorithm_frequency=1000;
  TargetConfig target;target.id="car";target.position={2,0,.3};Runner runner(c,{target});
  runner.schedule({pi/2,0,false,{}},1);
  std::vector<SensorPacket> captures;runner.image_sink=[&](const SensorPacket& p){captures.push_back(p);};
  Receiver receiver;auto& log=runner.run(30,receiver);
  check(captures.size()==3 && receiver.packets.size()==2,"Image capture or delivery frequency wrong");
  check(receiver.packets[0].capture_tick==0 && receiver.packets[0].ready_tick==11,"Image delay not rounded upwards");
  check(receiver.packets[0].image->bgr==captures[0].image->bgr,"Image was re-rendered at delivery");
  check(lights(*receiver.packets[0].image).count>0 && lights(*receiver.packets[1].image).count==0,"Image used command pose instead of capture-time actual pose");
  check(log.pending_observations==1,"Pending camera frames lost at end");
  for(auto& p:log.observations)check(!p.image,"RunLog retained full video pixels");
  auto bad=config();bad.image.light_width=.1;rejects([&]{Sensor s(bad,.001);});
  bad=config();bad.output="rgb";rejects([&]{Sensor s(bad,.001);});
  RunnerConfig heavy;heavy.sensor.latency=600;Receiver unused;TargetConfig t;t.id="heavy";
  rejects([&]{Runner r(heavy,{t});r.run(10000,unused);});
  bad=config();bad.calibration.width=65536;bad.calibration.height=65536;rejects([&]{Sensor s(bad,.001);});
}
}
int main(int argc,char** argv){try{if(argc>1)renderer=argv[1];projection_and_appearance();visibility();armor_identities();floating_plates_and_initial_view();timing_and_ownership();std::cout<<checks<<" image camera checks passed ("<<renderer<<")\n";}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
