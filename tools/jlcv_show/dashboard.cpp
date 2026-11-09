#include "window.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QGridLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QShortcut>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <thread>

namespace tools::jlcv_show {
namespace {
const QColor blue("#2563eb"),amber("#e58a35"),text("#334155"),muted("#64748b"),grid("#e6ecf3");
QString number(double value,int precision=2) { return QString::number(value,'f',precision); }
void card(QPainter& p,const QRect& bounds,QString title,QString subtitle={}) {
  p.setRenderHint(QPainter::Antialiasing);p.setPen(QColor("#e0e6ef"));p.setBrush(Qt::white);
  p.drawRoundedRect(QRectF(bounds).adjusted(.5,.5,-.5,-.5),10,10);
  auto f=p.font();f.setPointSize(11);f.setBold(true);p.setFont(f);p.setPen(text);
  p.drawText(18,27,title);f.setPointSize(9);f.setBold(false);p.setFont(f);p.setPen(muted);
  if(!subtitle.isEmpty())p.drawText(18,47,subtitle);
}
class Plot : public QWidget {
 public:
  Snapshot sample;int primary,secondary;QString title;
  Plot(QString name,int a,int b=-1):primary(a),secondary(b),title(std::move(name)) { setMinimumSize(360,160); }
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);const bool digital=primary==command_shoot;
    card(p,rect(),title,digital?"command.shoot · 0 关闭 / 1 发射":"角度 / ° · 最近 10 秒 · 自动缩放");
    const auto& first=sample.curves[primary];
    const std::vector<Point> empty;const auto& second=secondary<0?empty:sample.curves[secondary];
    const double end=std::max(2.,sample.time),start=std::max(0.,end-Channel::history_seconds);
    auto range=digital?Range{-.15,1.15}:value_range(first,second,start);
    QRectF area(64,67,width()-90,height()-102);
    auto position=[&](double t,double y){return QPointF(area.left()+(t-start)/(end-start)*area.width(),
      area.bottom()-(y-range.minimum)/(range.maximum-range.minimum)*area.height());};
    p.setFont(QFont(p.font().family(),8));
    for(int i=0;i<=4;++i) {
      double t=start+(end-start)*i/4.;double x=position(t,range.minimum).x();
      p.setPen(grid);p.drawLine(QPointF(x,area.top()),QPointF(x,area.bottom()));p.setPen(muted);
      p.drawText(QRectF(x-25,area.bottom()+8,50,16),Qt::AlignHCenter,number(t,1));
      if(!digital) {
        double y=range.minimum+(range.maximum-range.minimum)*i/4.;double py=position(t,y).y();
        p.setPen(grid);p.drawLine(QPointF(area.left(),py),QPointF(area.right(),py));p.setPen(muted);
        p.drawText(QRectF(2,py-8,53,16),Qt::AlignRight|Qt::AlignVCenter,number(y,range.maximum-range.minimum<2?2:1));
      }
    }
    if(digital)for(int value:{0,1}) {
      auto y=position(start,value).y();p.setPen(grid);p.drawLine(QPointF(area.left(),y),QPointF(area.right(),y));
      p.setPen(muted);p.drawText(QRectF(2,y-8,53,16),Qt::AlignRight|Qt::AlignVCenter,QString::number(value));
    }
    p.setPen(muted);p.drawText(QRectF(area.right()-48,area.bottom()+22,48,15),Qt::AlignRight,"时间 / s");
    auto curve=[&](const std::vector<Point>& points,QColor color) {
      p.save();p.setClipRect(area.adjusted(-1,-1,1,1));QPainterPath path;bool connected=false;
      double previous=0,previous_value=0;
      for(const auto& v:points) {
        if(v.time<start-.3)continue;
        if(!v.valid || !std::isfinite(v.value)){connected=false;continue;}
        auto q=position(v.time,v.value);
        if(!connected || v.time-previous>.5)path.moveTo(q);
        else if(digital){path.lineTo(position(v.time,previous_value));path.lineTo(q);}
        else path.lineTo(q);
        previous=v.time;previous_value=v.value;connected=true;
      }
      p.setBrush(Qt::NoBrush);p.setPen(QPen(color,1.8));p.drawPath(path);p.restore();
    };
    curve(second,amber);curve(first,blue);
    auto legend=[&](int x,QString name,const std::vector<Point>& points,QColor color) {
      p.setPen(QPen(color,2));p.drawLine(x,25,x+18,25);p.setPen(muted);
      QString value=points.empty() || !points.back().valid ? "—":number(points.back().value,digital?0:2);
      p.drawText(x+25,29,name+"  "+value);
    };
    if(!digital && width()>450) {
      legend(width()-322,"发送",first,blue);legend(width()-165,"反馈",second,amber);
    }else if(digital)legend(width()-190,"shoot",first,blue);
    if(first.empty() && second.empty()) {
      p.setPen(muted);p.drawText(area,Qt::AlignCenter,"等待数据");
    }
  }
};
class Camera : public QWidget {
 public: Snapshot sample;
  Camera(){setMinimumSize(250,190);}
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);card(p,rect(),"相机 / 检测结果");
    QRect area(12,45,width()-24,height()-57);p.fillRect(area,QColor("#161b22"));
    if(!sample.preview.empty()) {
      QImage image(sample.preview.data,sample.preview.cols,sample.preview.rows,int(sample.preview.step),QImage::Format_RGB888);
      auto rgb=image.rgbSwapped().scaled(area.size(),Qt::KeepAspectRatio,Qt::SmoothTransformation);
      p.drawImage(QPoint(area.center().x()-rgb.width()/2,area.center().y()-rgb.height()/2),rgb);
    }else{p.setPen(QColor("#94a3b8"));p.drawText(area,Qt::AlignCenter,"等待相机图像");}
  }
};
class World : public QWidget {
 public: Snapshot sample;
  World(){setMinimumSize(270,280);}
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);card(p,rect(),"估计装甲板 / 世界坐标",QString("%1 块 · x/y/z 单位 m").arg(sample.armors.size()));
    const QRectF area(20,62,width()-40,height()-132);
    const Eigen::Vector3d right=Eigen::Vector3d(1,1,0).normalized();
    const Eigen::Vector3d up=Eigen::Vector3d(-.5,.5,1).normalized();
    const auto raw=[&](const Eigen::Vector3d& v){return QPointF(v.dot(right),-v.dot(up));};
    std::vector<std::array<Eigen::Vector3d,4>> plates;
    QPointF lo(1e9,1e9),hi(-1e9,-1e9);Eigen::Vector3d center=Eigen::Vector3d::Zero();
    auto bound=[&](const Eigen::Vector3d& v){auto q=raw(v);lo.setX(std::min(lo.x(),q.x()));lo.setY(std::min(lo.y(),q.y()));
      hi.setX(std::max(hi.x(),q.x()));hi.setY(std::max(hi.y(),q.y()));};
    const bool fresh=sample.estimate_age>=0 && sample.estimate_age<.6;
    if(fresh)for(const auto& armor:sample.armors) {
      const auto c=armor.xyza.head<3>().eval();center+=c;
      Eigen::Vector3d r(-std::sin(armor.xyza[3]),std::cos(armor.xyza[3]),0),z=plate_up(armor);
      std::array<Eigen::Vector3d,4> corners{c-r*armor.width/2+z*armor.height/2,c+r*armor.width/2+z*armor.height/2,
        c+r*armor.width/2-z*armor.height/2,c-r*armor.width/2-z*armor.height/2};
      for(const auto& v:corners)bound(v);
      bound(c+plate_normal(armor)*.14);plates.push_back(corners);
    }
    if(fresh && sample.aim)bound(*sample.aim);
    if(plates.empty()) {
      p.setPen(muted);p.drawText(area,Qt::AlignCenter,sample.estimate_age>=.6?"估计已过期":"等待跟踪目标");
    }else {
      const auto middle=(lo+hi)*.5;
      const double scale=std::min(area.width()/std::max(.65,hi.x()-lo.x()),area.height()/std::max(.5,hi.y()-lo.y()))*.82;
      auto project=[&](const Eigen::Vector3d& v){return area.center()+(raw(v)-middle)*scale;};
      for(size_t i=0;i<plates.size();++i) {
        const auto& armor=sample.armors[i];auto c=armor.xyza.head<3>().eval();
        const auto n=plate_normal(armor);
        QPolygonF face;for(const auto& v:plates[i])face<<project(v);
        p.setPen(QPen(blue,1.5));p.setBrush(QColor("#dbeafe"));p.drawPolygon(face);
        p.drawLine(project(c),project(c+n*.14));p.setPen(text);
        p.drawText(project(c)+QPointF(6,-8),QString("P%1").arg(i+1));
      }
      if(sample.aim) {auto q=project(*sample.aim);p.setPen(QPen(amber,2));p.drawLine(q-QPointF(5,0),q+QPointF(5,0));p.drawLine(q-QPointF(0,5),q+QPointF(0,5));}
      center/=plates.size();p.setPen(muted);p.setFont(QFont(p.font().family(),8));
      p.drawText(16,height()-52,QString("中心  %1  /  %2  /  %3").arg(number(center.x()),number(center.y()),number(center.z())));
    }
    QPointF origin(34,height()-21);
    const Eigen::Vector3d axes[]{Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ()};
    const QColor colors[]{QColor("#dc3545"),QColor("#32966a"),blue};
    const char* labels[]{"X","Y","Z"};
    for(int i=0;i<3;++i){auto end=origin+raw(axes[i])*19;p.setPen(QPen(colors[i],1.6));p.drawLine(origin,end);p.drawText(end+QPointF(2,-1),labels[i]);}
    p.setPen(muted);p.drawText(98,height()-14,"蓝色：位姿 / 朝向   橙色：瞄准点");
  }
};
class Window : public QWidget {
 public:
  Channel& channel;Camera* camera;World* world;std::array<Plot*,3> plots;QLabel *status,*metrics;
  QPushButton* pause;QLabel* footer;bool paused=false;double displayed_time=0;
  explicit Window(Channel& c):channel(c) {
    setProperty("display_paused",false);setProperty("display_time",0.);
    setWindowTitle("JLCV / 实时调试");resize(1280,820);setMinimumSize(980,690);
    setStyleSheet("QWidget{background:#f8fafc;color:#334155;font-family:'Noto Sans CJK SC','sans-serif';font-size:12px;}"
      "QLabel#title{font-size:20px;font-weight:600;background:transparent;}"
      "QLabel#status{color:#2563eb;background:#eff6ff;border:1px solid #dbeafe;border-radius:6px;padding:8px 12px;}"
      "QLabel#metrics{background:white;border:1px solid #e0e6ef;border-radius:10px;padding:15px;line-height:1.6;}"
      "QPushButton#pause_display{background:white;border:1px solid #cbd5e1;border-radius:6px;padding:8px 16px;color:#2563eb;}"
      "QPushButton#pause_display:hover{background:#eff6ff;}"
      "QPushButton#pause_display:checked{background:#fff7ed;border-color:#fdba74;color:#9a3412;}");
    auto* layout=new QVBoxLayout(this);layout->setContentsMargins(20,18,20,18);layout->setSpacing(14);
    auto* header=new QHBoxLayout;auto* title=new QLabel("JLCV / 自瞄调试");title->setObjectName("title");header->addWidget(title);
    header->addStretch();status=new QLabel("正在启动");status->setObjectName("status");header->addWidget(status);
    pause=new QPushButton("暂停显示");pause->setObjectName("pause_display");pause->setCheckable(true);
    pause->setCursor(Qt::PointingHandCursor);pause->setToolTip("空格：暂停/继续显示。自瞄、通信和控制持续运行；继续后显示最新数据。");
    header->addWidget(pause);layout->addLayout(header);
    auto* body=new QHBoxLayout;body->setSpacing(14);auto* left=new QVBoxLayout;left->setSpacing(14);
    camera=new Camera;camera->setMaximumWidth(360);world=new World;world->setMaximumWidth(360);
    camera->setObjectName("camera_preview");world->setObjectName("world_estimates");
    metrics=new QLabel;metrics->setObjectName("metrics");metrics->setWordWrap(true);metrics->setMaximumWidth(360);
    left->addWidget(camera,3);left->addWidget(metrics,0);left->addWidget(world,4);body->addLayout(left,3);
    auto* curves=new QVBoxLayout;curves->setSpacing(14);
    plots={new Plot("Yaw",command_yaw,feedback_yaw),new Plot("Pitch",command_pitch,feedback_pitch),new Plot("发射指令",command_shoot)};
    plots[0]->setObjectName("yaw_plot");plots[1]->setObjectName("pitch_plot");plots[2]->setObjectName("shoot_plot");
    for(auto* plot:plots)curves->addWidget(plot,1);body->addLayout(curves,8);layout->addLayout(body,1);
    footer=new QLabel;footer->setStyleSheet("color:#64748b;background:transparent;");layout->addWidget(footer);
    const QString live_footer="实时刷新 · yaw 连续展开 · 坐标轴自动缩放 · 空格 暂停/继续显示 · Esc 关闭";
    footer->setText(live_footer);
    connect(pause,&QPushButton::toggled,this,[this,live_footer](bool checked){
      paused=checked;setProperty("display_paused",paused);
      pause->setText(paused?"继续实时":"暂停显示");
      footer->setText(paused?QString("显示已暂停于 %1 s · 自瞄 / 通信 / 控制继续运行 · 空格 继续实时 · Esc 关闭").arg(number(displayed_time,3)):live_footer);
      refresh();
    });
    auto* shortcut=new QShortcut(QKeySequence(Qt::Key_Space),this);
    connect(shortcut,&QShortcut::activated,pause,&QPushButton::click);
    auto* timer=new QTimer(this);connect(timer,&QTimer::timeout,this,[this]{refresh();});timer->start(33);refresh();
  }
  void refresh() {
    auto s=channel.snapshot();status->setText(QString::fromStdString(s.source)+" · "+QString::fromStdString(s.status));
    if(paused) {
      status->setText(status->text()+QString(" · 显示已暂停 %1 s").arg(number(displayed_time,3)));
      return;
    }
    displayed_time=s.time;setProperty("display_time",displayed_time);
    QString feedback=s.feedback_age<0?"暂无反馈":s.feedback_age>.5?"反馈已过期":QString("%1 ms").arg(number(s.feedback_age*1000,0));
    metrics->setText(QString("<b>跟踪</b>　%1<br><br>控制　%2　　发射　%3<br><br>反馈　%4")
      .arg(s.target.empty()?"暂无目标":QString::fromStdString(s.target),s.control?"开":"关",s.shoot?"1":"0",feedback));
    camera->sample=s;world->sample=s;camera->update();world->update();for(auto* plot:plots){plot->sample=s;plot->update();}
  }
  void closeEvent(QCloseEvent* e) override {
    if(property("worker_finished").toBool()){e->accept();return;}
    channel.stop();channel.status("正在停止相机 / 通信");refresh();e->ignore();
  }
  void keyPressEvent(QKeyEvent* e) override {if(e->key()==Qt::Key_Escape)close();else QWidget::keyPressEvent(e);}
};
}
std::unique_ptr<QWidget> make_window(Channel& c) { return std::make_unique<Window>(c); }
int run_dashboard(const std::function<int(Channel&)>& worker) {
  int argc=1;char name[]="jlcv";char* argv[]{name,nullptr};QApplication app(argc,argv);
  Channel channel;auto window=make_window(channel);window->show();
  std::atomic<bool> done{false};int result=1;std::exception_ptr error;
  std::thread vision([&]{try{result=worker(channel);}catch(...){error=std::current_exception();}done=true;});
  QTimer completion;QObject::connect(&completion,&QTimer::timeout,&app,[&]{
    if(done.load()){window->setProperty("worker_finished",true);app.quit();}
  });completion.start(30);
  app.exec();channel.stop();vision.join();
  if(error)try{std::rethrow_exception(error);}catch(const std::exception& e){std::cerr<<"[JLCV show] "<<e.what()<<'\n';}
  return result;
}
}
