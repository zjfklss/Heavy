#include "../configuration/configuration_editor.hpp"
#include "rm_sim/scenario.hpp"
#include <QApplication>
#include <iostream>
#include <stdexcept>

using namespace rm_sim;
namespace {
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
void near(double a,double b) { check(std::abs(a-b)<1e-8,"Unexpected target kinematics"); }
QJsonObject first(const QJsonObject& root) {
  return root["scene"].toObject()["targets"].toArray()[0].toObject();
}
TargetConfig parse(const QJsonObject& target) {
  return scenario::target(nlohmann::json::parse(QJsonDocument(target).toJson().toStdString()));
}
void trajectory(const QJsonObject& target) {
  const auto config=parse(target);
  config.validate();
  // Independent position/derivative expectations, including turning points and many periods.
  for (double t:{0.,pi/2,pi,3*pi/2,2*pi,1000.,10000.}) {
    const auto y=config.translation[1].at(t);
    near(y.position,-3*std::sin(t));
    near(y.velocity,-3*std::cos(t));
    near(y.acceleration,3*std::sin(t));
    near(y.jerk,3*std::cos(t));
    near(config.translation[0].at(t).position,0);
    near(config.translation[2].at(t).position,0);
    near(config.rotation.at(t).position,config.rotation.initial+7*t);
    near(config.rotation.at(t).velocity,7);
  }
}
}
int main(int argc,char** argv) {
  QApplication app(argc,argv);
  try {
    ui::ConfigurationEditor editor;
    auto* target=editor.target;
    const auto original=editor.configuration();
    const int preset=target->car_motion->findData("lateral_spin");
    check(preset>=0,"Missing car motion preset");
    target->car_motion->setCurrentIndex(preset);
    target->phase->setValue(30);
    auto saved=editor.configuration();
    trajectory(first(saved));
    near(parse(first(saved)).rotation.initial,pi/6);
    // Save/reload via JSON must keep both the selected option and all unrelated configuration.
    auto encoded=QJsonDocument(saved).toJson();
    editor.load(QJsonDocument::fromJson(encoded).object());
    check(target->lateral_spin(),"Motion preset was not restored");
    check(editor.configuration()==saved,"Motion preset changed on reload");
    target->car_motion->setCurrentIndex(0);
    target->my->kind->setCurrentIndex(target->my->kind->findData("velocity"));
    target->my->velocity->setValue(.8);
    near(parse(first(editor.configuration())).translation[1].at(10).position,8);

    // Import hidden offsets, phase and Z motion. Selecting the preset must replace all of them.
    auto root=original, scene=root["scene"].toObject(), car=first(root);
    car["translation"]=QJsonArray{QJsonObject{{"kind","velocity"},{"velocity",2},{"initial",1}},
      QJsonObject{{"kind","sinusoidal"},{"velocity",-3},{"acceleration",0},
        {"amplitude",3},{"frequency",1},{"phase",1.2},{"initial",2}},
      QJsonObject{{"kind","velocity"},{"velocity",.1}}};
    car["rotation"]=QJsonObject{{"kind","acceleration"},{"velocity",2},
      {"acceleration",.7},{"initial",.123456789012345}};
    car["tilt"]=.2123456789;
    const QJsonObject other{{"id","other"},{"kind","car"},{"position",QJsonArray{8,0,.3}}};
    scene["targets"]=QJsonArray{car,other}; root["scene"]=scene;
    editor.load(root);
    check(!target->lateral_spin(),"Custom motion misidentified as preset");
    target->car_motion->setCurrentIndex(preset);
    saved=editor.configuration();
    trajectory(first(saved));
    check(first(saved)["position"]==car["position"],"Preset changed target origin");
    check(first(saved)["tilt"]==car["tilt"],"Preset changed target geometry");
    check(saved["scene"].toObject()["targets"].toArray()[1]==other,"Preset changed other target");
    check(first(saved)["rotation"].toObject()["initial"]==car["rotation"].toObject()["initial"],
      "Preset lost initial orientation precision");
    editor.load(saved);
    check(target->lateral_spin() && editor.configuration()==saved,"Imported preset roundtrip failed");
    target->kind->setCurrentIndex(target->kind->findData("outpost"));
    check(!first(editor.configuration()).contains("translation"),"Car motion leaked into outpost");
    std::cout<<"Car lateral/spin preset: bounded trajectory, JSON roundtrip and imported motion replacement passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
