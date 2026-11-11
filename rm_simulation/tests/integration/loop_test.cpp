#include "rm_sim/runner.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace rm_sim;
namespace {
int checks=0;
void check(bool value,const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
void near(double a,double b,double tolerance=1e-8) { check(std::abs(a-b)<=tolerance,"Numeric mismatch"); }
void rejects(const std::function<void()>& fn) {
  bool failed=false; try { fn(); } catch (const std::invalid_argument&) { failed=true; }
  check(failed,"Invalid input accepted");
}
TargetConfig car() { TargetConfig c; c.id="target"; return c; }
Snapshot flat_surface() {
  Snapshot s;
  TargetState t{}; t.id="panel"; t.kind=TargetKind::car;
  Surface p; p.target_id=t.id; p.center={4,0,.3}; p.normal={-1,0,0};
  p.right={0,-1,0}; p.up={0,0,1}; p.width=.2; p.height=.1;
  t.surfaces.push_back(p); s.targets.push_back(t); return s;
}
void clocks_and_transforms() {
  for (double hz:{60.,144.}) {
    PeriodicClock clock(hz,.001);
    int samples=0;
    for (Tick tick=0;tick<10000;++tick) if (clock.take(tick)) {
      double ideal=samples/hz;
      check(tick*.001>=ideal-1e-10 && tick*.001-ideal<.001+1e-10,"Sampling drift");
      ++samples;
    }
    check(samples==int(10*hz),"Wrong periodic sample count");
  }
  near(quantize_delay(.00101,.001),2);
  near(quantize_delay(0,.001),0);
  Pose a{yaw_rotation(.7),{1,2,3}}, b{pitch_rotation(.3),{.1,.2,.3}};
  auto c=compose(a,b); Vec3 p{3,4,5};
  near(norm(c.apply(p)-a.apply(b.apply(p))),0);
  near(norm(c.inverse(c.apply(p))-p),0);
  rejects([]{PeriodicClock c(1001,.001);});
  rejects([]{Pose p; p.rotation.x={2,0,0}; p.validate();});
}
void sensors() {
  Actuator actuator({},.001);
  SensorConfig config; config.output="features"; config.latency=.0101;
  Sensor sensor(config,.001);
  auto truth=flat_surface();
  auto result=sensor.capture(truth,actuator.truth().camera_pose,0);
  check(result.packet.ready_tick==11 && result.packet.capture_tick==0,"Sensor timestamp quantization");
  check(result.packet.observations.size()==1,"Visible panel missing");
  near(result.packet.observations[0].pixels[0].x,617.5);
  near(result.packet.observations[0].pixels[0].y,500.75);
  check(result.truth.features[0].observation_index==0,"Truth association missing");
  truth.targets[0].surfaces[0].normal={1,0,0};
  check(sensor.capture(truth,actuator.truth().camera_pose,0).packet.observations.empty(),"Back face observed");
  truth=flat_surface(); truth.targets[0].surfaces[0].center.y=20;
  check(sensor.capture(truth,actuator.truth().camera_pose,0).packet.observations.empty(),"Out-of-image target observed");
  truth.targets[0].surfaces[0].center={-4,0,.3}; truth.targets[0].surfaces[0].normal={1,0,0};
  auto behind=sensor.capture(truth,actuator.truth().camera_pose,0);
  check(behind.truth.features[0].reason=="behind_camera","Behind-camera filtering");
  config.pixel_sigma=.5; config.seed=42;
  Sensor a(config,.001),b(config,.001); truth=flat_surface();
  auto pa=a.capture(truth,actuator.truth().camera_pose,0).packet;
  auto pb=b.capture(truth,actuator.truth().camera_pose,0).packet;
  near(pa.observations[0].pixels[0].x,pb.observations[0].pixels[0].x,0);
  near(pa.observations[0].covariance[0],.25);
  check(pa.observations[0].pixels[0].x!=617.5,"Noise not applied");
  config.dropout=1; config.false_positive_probability=0;
  Sensor dropped(config,.001);
  check(dropped.capture(truth,actuator.truth().camera_pose,0).packet.observations.empty(),"Dropout ignored");
  config.false_positive_probability=1;
  Sensor false_positive(config,.001);
  check(false_positive.capture(truth,actuator.truth().camera_pose,0).packet.observations.size()==1,"False positive missing");
  config=SensorConfig{}; config.output="features"; config.surface_occlusion=true;
  auto near_panel=truth.targets.front(); near_panel.id="near";
  near_panel.surfaces[0].target_id="near"; near_panel.surfaces[0].center.x=2;
  truth.targets.push_back(near_panel);
  Sensor occlusion(config,.001);
  auto occluded=occlusion.capture(truth,actuator.truth().camera_pose,0);
  check(occluded.packet.observations.size()==1 && occluded.truth.features[0].reason=="surface_occlusion","Surface occlusion failed");
  config.appearances["panel"]={"test-points","measured-armor",{{0,0,0}}};
  Sensor custom(config,.001);
  auto point=custom.capture(flat_surface(),actuator.truth().camera_pose,0);
  check(point.packet.observations[0].model_id=="test-points","Custom feature model missing");
  near(point.packet.observations[0].pixels[0].x,640);
  rejects([&]{sensor.capture(flat_surface(),actuator.truth().camera_pose,1);});
}
void actuators() {
  Actuator a({},.001);
  a.apply({0,0,0,{pi/2,.3,true,"one"}});
  auto s=a.truth();
  near(s.muzzle_pose.rotation.x.x,0);
  near(s.muzzle_pose.rotation.x.y,std::cos(.3));
  near(s.muzzle_pose.rotation.x.z,std::sin(.3));
  auto shots=a.launches(); check(shots.size()==1 && shots[0].tick==0,"Ideal launch missing");
  near(norm(shots[0].shot.velocity-s.muzzle_pose.rotation.x*23),0);
  a.apply({0,0,0,{0,0,true,"one"}});
  a.apply({1,0,0,{0,0,true,"one"}});
  check(a.launches().empty(),"Duplicate fire request generated a shot");
  ActuatorConfig delayed; delayed.fire_latency=.0101; delayed.fire_interval=.05;
  Actuator d(delayed,.001);
  d.apply({0,0,0,{0,0,true,"a"}}); d.apply({1,0,0,{0,0,true,"b"}});
  for (int i=0;i<11;++i) { check(d.launches().empty(),"Early launch"); d.advance(); }
  d.apply({2,11,11,{.5,0,false,{}}});
  auto first=d.launches(); check(first.size()==1 && first[0].tick==11,"Fire latency wrong");
  near(first[0].shot.velocity.y,23*std::sin(.5));
  for (int i=0;i<50;++i) { check(d.launches().empty(),"Fire rate exceeded"); d.advance(); }
  check(d.launches().size()==1,"Queued shot missing");
  ActuatorConfig limits; limits.ideal=false; limits.max_speed=1; limits.max_acceleration=5;
  limits.initial_yaw=pi-.02;
  limits.pitch_offset={.2,0,0}; limits.muzzle_mount.translation={.1,0,0};
  Actuator dynamic(limits,.001);
  dynamic.apply({0,0,0,{-pi+.02,.4,false,{}}});
  double previous=0;
  for (int i=0;i<1000;++i) {
    dynamic.advance(); auto t=dynamic.truth();
    check(std::abs(t.yaw_velocity)<=1+1e-10 && std::abs(t.pitch_velocity)<=1+1e-10,"Speed cap exceeded");
    check(std::abs(t.pitch_velocity-previous)<=.005+1e-10,"Acceleration cap exceeded");
    previous=t.pitch_velocity;
    if (i==5) {
      check(t.yaw_velocity>0,"Yaw wrap took the long path");
      auto copy=dynamic; copy.advance();
      auto finite_velocity=(copy.truth().muzzle_pose.translation-t.muzzle_pose.translation)*1000;
      check(norm(finite_velocity-t.muzzle_velocity)<.002,"Muzzle inherited velocity/offset wrong");
    }
  }
  near(dynamic.truth().pitch,.4,1e-4);
  ActuatorConfig bounded; bounded.wrap_yaw=false; bounded.yaw_min=-.2; bounded.yaw_max=.2;
  Actuator clamp(bounded,.001); clamp.apply({0,0,0,{5,5,false,{}}});
  near(clamp.truth().yaw,.2); near(clamp.truth().pitch,bounded.pitch_max);
  ActuatorConfig noisy; noisy.encoder_sigma=.01; noisy.feedback_latency=.0021;
  Actuator noisy_a(noisy,.001),noisy_b(noisy,.001);
  auto feedback=noisy_a.feedback(); near(feedback.yaw,noisy_b.feedback().yaw,0);
  check(feedback.ready_tick==3 && feedback.yaw!=0,"Encoder feedback model missing");
  near(noisy_a.truth().yaw,0);
  noisy.muzzle_speed_sigma=1;
  Actuator stream_a(noisy,.001),stream_b(noisy,.001);
  for (int i=0;i<100;++i) stream_a.feedback();
  stream_a.apply({0,0,0,{0,0,true,"shot"}});
  stream_b.apply({0,0,0,{0,0,true,"shot"}});
  near(stream_a.launches()[0].shot.velocity.x,stream_b.launches()[0].shot.velocity.x,0);
  rejects([&]{noisy_a.apply({0,0,1,{}});});
}
struct OneCommand : Algorithm {
  std::vector<AlgorithmInput> inputs;
  std::string version() const override { return "test/1"; }
  std::vector<ControlRequest> update(const AlgorithmInput& in) override {
    inputs.push_back(in);
    if (in.tick==0) return {{.4,0,true,"once"}};
    return {};
  }
};
void runners() {
  RunnerConfig cfg; cfg.sensor.output="features"; cfg.world.gravity={0,0,0}; cfg.algorithm_frequency=1000;
  cfg.sensor.frequency=1000; cfg.sensor.latency=.01; cfg.actuator.feedback_latency=.003;
  cfg.actuator.feedback_frequency=1000;
  cfg.log_frequency=1000;
  OneCommand algorithm;
  Runner runner(cfg,{car()}); const auto& log=runner.run(30,algorithm);
  check(log.commands.size()==1 && log.commands[0].due_tick==1,"Zero-delay command did not wait one tick");
  check(log.launches.size()==1 && log.launches[0].tick==1,"Actual launch ordering wrong");
  near(log.launches[0].shot.velocity.y,23*std::sin(.4));
  check(algorithm.inputs[0].observations.empty() && algorithm.inputs[0].feedback.empty(),"Future messages leaked");
  check(algorithm.inputs[10].observations.size()==1,"Delayed packet missing");
  const auto& received=algorithm.inputs[10].observations[0];
  check(received.capture_tick==0 && received.ready_tick==10,"Capture timestamp overwritten on delivery");
  near(received.observations[0].pixels[0].x,log.observations[0].observations[0].pixels[0].x,0);
  near(log.annotations[0].camera_pose.rotation.z.x,1);
  near(log.annotations[1].camera_pose.rotation.z.x,std::cos(.4));
  check(log.pending_observations==10 && log.pending_feedback==3,"Pending messages not accounted");
  for (const auto& f:log.frames) near(f.world.time,f.actuator.tick*.001);
  rejects([&]{runner.run(1,algorithm);});
  cfg=RunnerConfig{}; cfg.sensor.output="features"; cfg.world.gravity={0,0,0}; cfg.actuator.initial_yaw=.15;
  PixelServo servo;
  Runner baseline(cfg,{car()}); const auto& ideal=baseline.run(2000,servo);
  check(ideal.launches.size()>=4 && ideal.hits.size()>=4,"Ideal closed loop did not hit target");
  check(std::all_of(ideal.hits.begin(),ideal.hits.end(),[](const auto& h){return h.effective;}),"Ideal hits ineffective");
  cfg.sensor.pixel_sigma=.2; cfg.sensor.latency=.02; cfg.actuator.ideal=false;
  cfg.actuator.command_latency=.005; cfg.actuator.feedback_latency=.005;
  PixelServo s1,s2; Runner r1(cfg,{car()}),r2(cfg,{car()});
  const auto& l1=r1.run(1000,s1); const auto& l2=r2.run(1000,s2);
  check(l1.commands.size()==l2.commands.size() && l1.launches.size()==l2.launches.size(),"Closed loop nondeterministic");
  for (size_t i=0;i<l1.commands.size();++i) near(l1.commands[i].request.yaw,l2.commands[i].request.yaw,0);
  cfg.sensor.enabled=false; cfg.actuator.feedback_enabled=false;
  OneCommand standalone; Runner disabled(cfg,{car()}); const auto& disabled_log=disabled.run(20,standalone);
  check(disabled_log.observations.empty() && disabled_log.feedback.empty() && disabled_log.launches.size()==1,
    "Disabled sensors prevent standalone actuator use");
  rejects([&]{Runner invalid(cfg,{car()},{{0,"stop_outpost","target"}});});
  cfg.actuator.fire_latency=1;
  OneCommand waiting; Runner queued(cfg,{car()});
  check(queued.run(20,waiting).pending_launches==1,"Pending fire request lost at run end");
  // Changing sensor rate/seed must not perturb the world's random target motion.
  auto moving=car(); moving.translation[1].kind=MotionKind::random;
  RunnerConfig ca,cb; ca.sensor.output=cb.sensor.output="features"; ca.sensor.frequency=60; cb.sensor.frequency=144; cb.sensor.seed=999;
  OneCommand aa,ab; Runner ra(ca,{moving}),rb(cb,{moving});
  const auto& la=ra.run(100,aa); const auto& lb=rb.run(100,ab);
  near(la.frames.back().world.targets[0].position.y,lb.frames.back().world.targets[0].position.y,0);
}
}
int main() {
  try { clocks_and_transforms(); sensors(); actuators(); runners(); std::cout<<checks<<" checks passed\n"; }
  catch (const std::exception& e) { std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n'; return 1; }
}
