#include "tools/jlcv_show/window.hpp"
#include <QApplication>
#include <QEventLoop>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace tools::jlcv_show;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
constexpr double radians=3.14159265358979323846/180;
std::vector<ArmorPose> plates(int count) {
  std::vector<ArmorPose> result;
  for(int i=0;i<count;++i) {
    double a=i*2*3.14159265358979323846/count+.5;
    result.push_back({{4+.25*std::cos(a),.7+.25*std::sin(a),.4+(i%2)*.05,a}});
  }
  return result;
}
void history_checks() {
  Channel c;c.reset("test");
  c.command(0,179*radians,0,true,false);c.feedback(0,-179*radians,0);
  c.command(.1,-178*radians,0,true,true);c.feedback(.1,179.5*radians,0);
  auto s=c.snapshot();
  check(std::abs(s.curves[command_yaw].back().value-182)<1e-8,"Yaw wrap created a 360-degree command jump");
  check(std::abs(s.curves[feedback_yaw].front().value-181)<1e-8,"Feedback used a different yaw branch");
  c.command(.05,0,0,true,false);
  check(c.snapshot().curves[command_yaw].size()==2,"Out-of-order commands rewrote history");
  c.command(.2,0,0,false,false);s=c.snapshot();
  check(!s.curves[command_yaw].back().valid && s.curves[command_shoot].back().value==0,
    "Disabled command was plotted as an actual zero-angle request");
  auto range=value_range({{0,999,true},{1,.1,true},{2,.2,true}}, {},1);
  check(range.minimum<.1 && range.maximum>.2 && range.maximum<1,"Adaptive axes included old outliers");
  range=value_range({{0,5,true}}, {},0);
  check(range.minimum<5 && range.maximum>5,"Constant series produced a zero-sized axis");
  c.reset("bounded");
  for(int i=0;i<20000;++i)c.feedback(i*.0001,0,0);
  check(c.snapshot().curves[feedback_yaw].size()<=Channel::maximum_points,"Unbounded plot history");
  c.feedback(40,0,0);s=c.snapshot();
  check(s.curves[feedback_yaw].size()<=2,"Old samples were not evicted");
  c.command(std::numeric_limits<double>::quiet_NaN(),0,0,true,true);
  check(c.snapshot().curves[command_shoot].empty(),"Invalid clock reached a plot");
  c.estimate(plates(4),Eigen::Vector3d(4,.6,.45),"three",{});
  check(c.snapshot().armors.size()==4,"Estimated car geometry was lost");
  c.estimate(plates(3),std::nullopt,"outpost",{});
  check(c.snapshot().armors.size()==3,"Outpost was fabricated as four plates");
  ArmorPose pose;pose.xyza<<3,0,.4,0;pose.pitch=-15*radians;
  const auto n=plate_normal(pose),up=plate_up(pose);
  check(n.x()<0 && n.z()<0,"Outpost receiving-face normal is reversed");
  check(std::abs(n.dot(up))<1e-12 && std::abs(n.norm()-1)<1e-12,"Plate display basis is not orthogonal");
  pose.pitch=15*radians;check(plate_normal(pose).z()>0,"Car/outpost plate tilts were conflated");
  c.reset("new session");s=c.snapshot();
  check(s.armors.empty() && s.curves[feedback_yaw].empty() && s.feedback_age<0,"Session restart retained old estimates");
}
void pause_checks(const std::filesystem::path& directory) {
  auto refresh=[] {QEventLoop loop;QTimer::singleShot(80,&loop,&QEventLoop::quit);loop.exec();};
  Channel c;c.reset("暂停显示测试");c.status("已连接");
  for(int i=0;i<=200;++i) {
    const double time=i*.01;
    c.feedback(time,std::sin(time)*.1,-.1);c.command(time,std::sin(time+.05)*.1,-.09,true,i%20<5);
  }
  c.estimate(plates(4),Eigen::Vector3d(4,.6,.45),"three",cv::Mat(240,320,CV_8UC3,cv::Scalar(28,28,28)));
  auto window=make_window(c);window->show();refresh();
  auto* pause=window->findChild<QPushButton*>("pause_display");
  check(pause && pause->isCheckable(),"Pause button missing");
  pause->click();refresh();
  check(window->property("display_paused").toBool() && !c.stopping(),"Pause stopped the worker");
  const double frozen=window->property("display_time").toDouble();
  std::vector<std::pair<QWidget*,QImage>> panels;
  for(const char* name:{"yaw_plot","pitch_plot","shoot_plot","camera_preview","world_estimates"}) {
    auto* panel=window->findChild<QWidget*>(name);check(panel,"Display panel missing");
    panels.push_back({panel,panel->grab().toImage()});
  }
  const auto metrics=window->findChild<QLabel*>("metrics")->text();
  // Continue receiving more than the entire history window on a worker thread.
  std::thread publisher([&]{
    for(int i=1;i<=20000;++i) {
      const double time=2+i*.001;
      c.feedback(time,.4,-.2);c.command(time,.5,-.3,true,true);
    }
    c.estimate(plates(3),std::nullopt,"outpost",cv::Mat(240,320,CV_8UC3,cv::Scalar(170,100,30)));
    c.status("等待下次连接");
  });publisher.join();refresh();
  const auto live=c.snapshot();
  check(live.time>frozen+Channel::history_seconds && live.curves[command_yaw].size()<=Channel::maximum_points,
    "Paused display blocked or expanded the live history");
  check(window->property("display_time").toDouble()==frozen,"Paused time axis advanced");
  for(const auto& panel:panels)check(panel.first->grab().toImage()==panel.second,"Paused panel changed");
  check(window->findChild<QLabel*>("metrics")->text()==metrics,"Paused values changed");
  check(window->findChild<QLabel*>("status")->text().contains("等待下次连接"),"Pause hid live connection status");
  check(window->grab().save(QString::fromStdString((directory/"paused.png").string())),"Could not save paused preview");
  pause->click();refresh();
  check(!window->property("display_paused").toBool() && window->property("display_time").toDouble()==live.time,
    "Resume did not jump to latest data");
  check(panels.front().first->grab().toImage()!=panels.front().second,"Resume kept frozen curves");
  check(window->grab().save(QString::fromStdString((directory/"resumed.png").string())),"Could not save resumed preview");
  pause->click();c.reset("新会话");c.command(.3,0,0,true,false);refresh();
  check(window->property("display_time").toDouble()==live.time,"Reconnect destroyed frozen display");
  pause->click();refresh();
  check(window->property("display_time").toDouble()==.3,"Resume displayed the previous session");
  pause->click();window->close();check(c.stopping(),"Paused window could not stop the worker");
}
}
int main(int argc,char** argv) {
  try {
    if(argc>1 && std::string(argv[1])=="--worker-exit")
      return run_dashboard([](Channel& c){c.status("done");return 0;});
    QApplication app(argc,argv);history_checks();
    const std::filesystem::path directory=argc>1?argv[1]:"/tmp/jlcv-show-test";
    std::filesystem::create_directories(directory);
    pause_checks(directory);
    for(const auto* scenario:{"tracking","empty","outpost"}) {
      Channel c;c.reset("仿真 / SP");c.status("已连接");
      if(std::string(scenario)!="empty") {
        for(int i=0;i<800;++i) {
          double t=i*.01;
          c.feedback(t,std::remainder(179+4*std::sin(t),360.)*radians,(-8+.6*std::sin(t*1.8))*radians);
          c.command(t,std::remainder(179+4*std::sin(t+.05),360.)*radians,(-8+.6*std::sin(t*1.8+.06))*radians,true,i%90<7);
        }
        cv::Mat image(240,320,CV_8UC3,cv::Scalar(28,28,28));
        c.estimate(plates(std::string(scenario)=="outpost"?3:4),Eigen::Vector3d(4,.6,.45),scenario,image);
      }
      auto window=make_window(c);window->show();app.processEvents();
      auto path=(directory/(std::string(scenario)+".png")).string();
      check(window->grab().save(QString::fromStdString(path)),"Could not render dashboard screenshot");
      QImage saved(QString::fromStdString(path));
      check(saved.width()>=980 && saved.height()>=690,"Dashboard layout collapsed");
      window->close();check(c.stopping(),"Closing UI did not request worker shutdown");
    }
    std::cout<<"Dashboard timing, wrapped yaw, adaptive bounds, bounded memory, session reset and offscreen UI passed\n";
    return 0;
  } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
