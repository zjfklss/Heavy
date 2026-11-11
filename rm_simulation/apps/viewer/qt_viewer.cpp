#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QListView>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QScrollArea>
#include <QSaveFile>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QProcess>
#include <QPushButton>
#include <QSlider>
#include <QPointer>
#include <QtEndian>
#include <QSplitter>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>

#include "rm_sim/world.hpp"
#include "rendering/models.hpp"
#include "rendering/mesh_renderer.hpp"
#include "configuration/configuration_editor.hpp"

using namespace rm_sim;
namespace
{
QString q(const std::string &s) { return QString::fromStdString(s); }
QString number(double v, int decimals = 2) { return QString::number(v, 'f', decimals); }
Vec3 vector(const QJsonValue &v)
{
  const auto a = v.toArray();
  if (a.size() != 3) throw std::runtime_error("Invalid vector in trace");
  for (const auto &x : a)
    if (!x.isDouble() || !std::isfinite(x.toDouble()))
      throw std::runtime_error("Non-finite vector in trace");
  return {a[0].toDouble(), a[1].toDouble(), a[2].toDouble()};
}
QString label(TargetKind k)
{
  switch (k) {
    case TargetKind::car:
      return "四装甲板车";
    case TargetKind::outpost:
      return "前哨站";
    case TargetKind::small_rune:
      return "小能量机关";
    case TargetKind::big_rune:
      return "大能量机关";
  }
  return "未知";
}
struct CameraFrame { double capture=0,ready=0; QString path; };
struct Trace
{
  QString name;
  std::vector<Snapshot> frames;
  std::vector<Hit> hits;
  std::vector<CameraFrame> camera;
};
Hit parse_hit(const QJsonObject& h)
{
  Hit hit;
  hit.time=h["time"].toDouble();hit.point=vector(h["point"]);
  hit.shot_id=h["shot_id"].toString().toStdString();
  hit.target_id=h["target_id"].toString().toStdString();hit.plate_id=h["plate_id"].toInt();
  hit.ring=h["ring"].toInt();hit.normal_speed=h["normal_speed"].toDouble();
  hit.effective=h["effective"].toBool();hit.reason=h["reason"].toString().toStdString();
  if(h["plate_point"].isArray())hit.plate_point=vector(h["plate_point"]);
  return hit;
}
Snapshot parse_snapshot(const QJsonObject& frame)
{
    Snapshot s;
    s.time = frame.value("time").toDouble(-1);
    if (s.time < 0 || !std::isfinite(s.time))
      throw std::runtime_error("Trace times must be nonnegative and strictly increasing");
    for (const auto &item : frame.value("targets").toArray()) {
      const auto t = item.toObject();
      TargetState target;
      target.id = t.value("id").toString().toStdString();
      target.armor_marking=t.value("armor_marking").toString("3").toStdString();
      const auto kind = t.value("kind").toString();
      if (kind == "car")
        target.kind = TargetKind::car;
      else if (kind == "outpost")
        target.kind = TargetKind::outpost;
      else if (kind == "small_rune")
        target.kind = TargetKind::small_rune;
      else if (kind == "big_rune")
        target.kind = TargetKind::big_rune;
      else
        throw std::runtime_error("Unknown target kind in trace");
      target.position = vector(t.value("position"));
      target.velocity = vector(t.value("velocity"));
      target.acceleration = vector(t.value("acceleration"));
      target.jerk = vector(t.value("jerk"));
      const auto r = t.value("rotation").toArray();
      if (r.size() != 4) throw std::runtime_error("Invalid rotation state");
      target.rotation = {r[0].toDouble(), r[1].toDouble(), r[2].toDouble(), r[3].toDouble()};
      target.activation = t.value("activation").toString().toStdString();
      target.completed_groups = t.value("completed_groups").toInt();
      target.accepted_hits = t.value("accepted_hits").toInt();
      for (const auto &v : t.value("surfaces").toArray()) {
        const auto p = v.toObject();
        Surface surface;
        surface.target_id = target.id;
        surface.plate_id = p.value("plate_id").toInt();
        surface.center = vector(p.value("center"));
        surface.normal = vector(p.value("normal"));
        surface.right = vector(p.value("right"));
        surface.up = vector(p.value("up"));
        surface.width = p.value("width").toDouble();
        surface.height = p.value("height").toDouble();
        surface.disk_radius = p.value("disk_radius").toDouble();
        surface.active = p.value("active").toBool();
        target.surfaces.push_back(surface);
      }
      s.targets.push_back(target);
    }
    for (const auto &v : frame.value("projectiles").toArray()) {
      const auto p = v.toObject();
      s.projectiles.push_back({p.value("id").toString().toStdString(), vector(p.value("position")),
                               vector(p.value("velocity"))});
    }
  return s;
}
Trace load_trace(const QString &path)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot open trace");
  if (file.size() > 256 * 1024 * 1024)
    throw std::runtime_error("Trace exceeds 256 MiB viewer limit");
  QJsonParseError error;
  const auto doc = QJsonDocument::fromJson(file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject())
    throw std::runtime_error("Invalid JSON trace");
  const auto root = doc.object();
  if (root.value("schema_version").toInt() != 1)
    throw std::runtime_error("Unsupported trace version");
  Trace trace;
  trace.name = root.value("scenario").toObject().value("name").toString("仿真轨迹");
  for(const auto& value:root["algorithm_channels"].toObject()["observations"].toArray()) {
    auto packet=value.toObject(),image=packet["image"].toObject();
    if(image.isEmpty()) continue;
    auto relative=image["file"].toString();
    if(relative.isEmpty() || QDir::isAbsolutePath(relative) || relative.split('/').contains(".."))
      throw std::runtime_error("Invalid camera image reference");
    double capture=packet["capture_time"].toDouble(-1),ready=packet["ready_time"].toDouble(-1);
    if(capture<0 || ready<capture || (!trace.camera.empty() && capture<=trace.camera.back().capture))
      throw std::runtime_error("Invalid camera image timestamps");
    trace.camera.push_back({capture,ready,QFileInfo(path).dir().filePath(relative)});
  }
  for (const auto &f : root.value("frames").toArray()) {
    auto snapshot=parse_snapshot(f.toObject());
    if(!trace.frames.empty() && snapshot.time<=trace.frames.back().time)
      throw std::runtime_error("Trace times must advance");
    trace.frames.push_back(std::move(snapshot));
  }
  if (trace.frames.empty()) throw std::runtime_error("Trace has no frames");
  for (const auto &v : root.value("hits").toArray()) {
    auto hit=parse_hit(v.toObject());
    // Legacy traces do not contain the exact contact-frame coordinates.
    // Recover an approximate attachment from their nearest recorded snapshot.
    if(!hit.plate_point) {
      const auto closest=std::min_element(trace.frames.begin(),trace.frames.end(),[&](const auto& a,const auto& b){
        return std::abs(a.time-hit.time)<std::abs(b.time-hit.time);});
      for(const auto& target:closest->targets)if(target.id==hit.target_id)
        for(const auto& plate:target.surfaces)if(plate.plate_id==hit.plate_id) {
          const auto d=hit.point-plate.center;
          hit.plate_point=Vec3{dot(d,plate.right),dot(d,plate.up),dot(d,plate.normal)};
        }
    }
    trace.hits.push_back(hit);
  }
  return trace;
}

// A native Qt software renderer. Geometry is projected in 3D with a CPU depth buffer;
// it intentionally has no OpenGL/Qt3D requirement and performs no physics.
class Scene : public QWidget
{
public:
  const Trace *trace = nullptr;
  int frame = 0, target = -1;
  bool trails = true, normals = false;
  double yaw = pi + .3, elevation = .43, distance = 12;
  Scene()
  {
    setMinimumSize(480, 360);
    setMouseTracking(true);
    hit_rate_=new QLabel(this);
    hit_rate_->setObjectName("live_hit_rate");
    hit_rate_->setAttribute(Qt::WA_TransparentForMouseEvents);
    hit_rate_->move(14,14);
    hit_rate_->setStyleSheet("QLabel{background:transparent;border:0;padding:0;color:#475569;"
      "font-size:12px;font-family:'Noto Sans CJK SC','sans-serif';}");
    hit_rate_->setToolTip("准确率：本轮累计有效命中数 ÷ 已发射弹丸数（含飞行中弹丸）\n"
      "DPS：最近 5 秒有效命中数 × 20 点伤害 ÷ 5 秒；重新运行时清零");
    hit_rate_->hide();
  }
  void hit_statistics(uint64_t shots,uint64_t accepted,double dps)
  {
    const QString percentage=shots ? number(100.*double(accepted)/double(shots),1)+"%" : "—";
    hit_rate_->setText(QString("准确率 %1  ·  DPS %2").arg(percentage).arg(number(dps,1)));
    hit_rate_->adjustSize();
    hit_rate_->show();
  }
  void hide_hit_statistics() {hit_rate_->hide();}
  double target_distance() const
  {
    if (!trace || target < 0 || target >= static_cast<int>(trace->frames[frame].targets.size()))
      return 12;
    const auto kind = trace->frames[frame].targets[target].kind;
    return kind == TargetKind::car ? 1.7 : (kind == TargetKind::outpost ? 3.2 : 4.5);
  }
  void view(int mode)
  {
    yaw = pi + (mode == 0 ? .3 : 0);
    elevation = mode == 0 ? .43 : (mode == 1 ? pi / 2 - .001 : 0);
    distance = target < 0 ? 12 : target_distance();
    update();
  }

protected:
  void mousePressEvent(QMouseEvent *e) override
  {
    if (e->button() == Qt::LeftButton) {
      drag_ = true;
      last_ = e->pos();
    }
  }
  void mouseReleaseEvent(QMouseEvent *) override { drag_ = false; }
  void mouseMoveEvent(QMouseEvent *e) override
  {
    if (!drag_) return;
    auto delta = e->pos() - last_;
    last_ = e->pos();
    yaw -= delta.x() * .006;
    elevation = std::clamp(elevation + delta.y() * .006, -.2, 1.569);
    update();
  }
  void wheelEvent(QWheelEvent *e) override
  {
    distance = std::clamp(distance * std::exp(-e->angleDelta().y() * .001), 1.0, 50.0);
    update();
    e->accept();
  }
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), Qt::white);
    if (!trace) {
      p.setPen(QColor("#64748b"));
      p.drawText(rect(), Qt::AlignCenter, "选择场景，启动实时仿真");
      return;
    }
    const auto &s = trace->frames.at(frame);
    Vec3 focus{5, 0, 1.2};
    if (target >= 0 && target < static_cast<int>(s.targets.size()))
      focus = s.targets[target].position + Vec3{0, 0,
        s.targets[target].kind == TargetKind::outpost ? .8 :
        (s.targets[target].kind == TargetKind::car ? -.07 : -.5)};
    const Vec3 eye =
      focus + Vec3{distance * std::cos(elevation) * std::cos(yaw),
                   distance * std::cos(elevation) * std::sin(yaw), distance * std::sin(elevation)};
    const auto forward = unit(focus - eye), right = unit(cross(forward, {0, 0, 1})),
               up = cross(right, forward);
    const double scale = std::min(width(), height()) * 1.1;
    auto project = [&](Vec3 v) -> std::optional<QPointF> {
      auto d = v - eye;
      double z = dot(d, forward);
      if (z <= .05) return {};
      return QPointF(width() / 2.0 + scale * dot(d, right) / z,
                     height() / 2.0 - scale * dot(d, up) / z);
    };
    auto line = [&](Vec3 a, Vec3 b, const QColor &color, double width = 1) {
      auto pa = project(a), pb = project(b);
      if (!pa || !pb) return;
      p.setPen(QPen(color, width));
      p.drawLine(*pa, *pb);
    };
    auto point = [&](Vec3 v, const QColor &color, double r = 3) {
      auto q = project(v);
      if (!q) return;
      p.setPen(Qt::NoPen);
      p.setBrush(color);
      p.drawEllipse(*q, r, r);
    };
    auto text = [&](Vec3 v, const QString &label) {
      auto q = project(v);
      if (!q) return;
      p.setPen(QColor("#475569"));
      p.drawText(*q + QPointF(6, -8), label);
    };
    for (int n = -12; n <= 16; ++n) {
      line({double(n), -12, 0}, {double(n), 12, 0}, QColor("#e2e8f0"));
      line({-12, double(n), 0}, {16, double(n), 0}, QColor("#e2e8f0"));
    }
    line({0, 0, 0}, {1, 0, 0}, QColor("#dc3545"), 2);
    line({0, 0, 0}, {0, 1, 0}, QColor("#32966a"), 2);
    line({0, 0, 0}, {0, 0, 1}, QColor("#2563eb"), 2);
    text({1, 0, 0}, "X");
    text({0, 1, 0}, "Y");
    text({0, 0, 1}, "Z");
    if (trails)
      for (size_t it = 0; it < s.targets.size(); ++it) {
        for (int i = std::max(3, frame - 120); i <= frame; i += 3) {
          const auto &a = trace->frames[i - 3].targets;
          const auto &b = trace->frames[i].targets;
          if (it < a.size() && it < b.size())
            line(a[it].position, b[it].position, QColor("#94a3b8"), 1.5);
        }
      }
    if (trails) {
      std::map<std::string, Vec3> previous;
      for (int i = std::max(0, frame - 45); i <= frame; ++i)
        for (const auto &b : trace->frames[i].projectiles) {
          auto it = previous.find(b.id);
          if (it != previous.end()) line(it->second, b.position, QColor("#b77908"), 1.5);
          previous[b.id] = b.position;
        }
    }
    visual::Model model;
    for (size_t i = 0; i < s.targets.size(); ++i)
      model.target(s.targets[i], QColor(i % 2 == 0 ? "#e53945" : "#2563eb"));
    model.impact_marks(s,trace->hits);
    p.drawImage(0, 0, visual::render(model, width(), height(), eye, forward, right, up, scale));
    for (const auto &t : s.targets) {
      double height = t.kind == TargetKind::outpost ? 1.7 :
                      (t.kind == TargetKind::car ? .18 : 1.0);
      text(t.position + Vec3{0, 0, height}, q(t.id));
      if (normals)
        for (const auto &a : t.surfaces)
          line(a.center, a.center + a.normal * .22, QColor("#7c3aed"), 1.3);
    }
    for (const auto &b : s.projectiles) point(b.position, QColor("#b77908"), 3.5);
  }

private:
  QLabel* hit_rate_;
  bool drag_ = false;
  QPoint last_;
};

class CameraView : public QWidget {
 public:
  CameraView() { setObjectName("camera_view");setMinimumSize(320,180); }
  void display(const CameraFrame* frame) {
    if(!frame) { image_=QImage();path_.clear();caption_="相机：尚无就绪图像";update();return; }
    if(path_!=frame->path) { path_=frame->path;image_.load(path_); }
    caption_=image_.isNull() ? "缺少相机图像，请保留轨迹旁的 .images 文件夹" :
      QString("相机 · %1 × %2 · 采样 %3 s · 就绪 %4 s").arg(image_.width()).arg(image_.height()).arg(frame->capture,0,'f',3).arg(frame->ready,0,'f',3);
    update();
  }
  void display_live(const QByteArray& pixels,int width,int height,double capture,double ready,const QString& renderer) {
    image_=QImage(reinterpret_cast<const uchar*>(pixels.constData()),width,height,width*3,QImage::Format_BGR888).copy();
    path_.clear();caption_=QString("实时相机 · %1 × %2 · %3 · 采样 %4 s · 就绪 %5 s")
      .arg(width).arg(height).arg(renderer).arg(capture,0,'f',3).arg(ready,0,'f',3);update();
  }
  bool has_image() const { return !image_.isNull(); }
 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);painter.fillRect(rect(),QColor("#15191d"));
    if(!image_.isNull()) {
      auto size=image_.size();size.scale(width(),std::max(1,height()-28),Qt::KeepAspectRatio);
      QRect destination(QPoint((width()-size.width())/2,28+(height()-28-size.height())/2),size);
      painter.setRenderHint(QPainter::SmoothPixmapTransform);painter.drawImage(destination,image_);
    }
    painter.fillRect(0,0,width(),28,QColor("#f1f3f5"));painter.setPen(QColor("#334155"));
    painter.drawText(QRect(10,0,width()-20,28),Qt::AlignVCenter,caption_);
  }
 private:
  QImage image_;QString path_,caption_;
};

class Window : public QMainWindow
{
public:
  Window()
  {
    setWindowTitle("仿真");
    resize(1280, 800);
    auto *root = new QWidget;
    auto *layout = new QVBoxLayout(root);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    auto *run = new QPushButton("导入场景");
    auto *split = new QSplitter;
    scene_ = new Scene;
    auto* scene_views=new QSplitter(Qt::Vertical);
    scene_views->addWidget(scene_);camera_=new CameraView;scene_views->addWidget(camera_);
    scene_views->setStretchFactor(0,3);scene_views->setStretchFactor(1,2);camera_->hide();
    split->addWidget(scene_views);
    auto *sidebar = new QWidget;
    sidebar->setMinimumWidth(340);
    sidebar->setMaximumWidth(400);
    auto *side = new QVBoxLayout(sidebar);
    side->setContentsMargins(12, 12, 12, 12);
    auto *pages = new QStackedWidget;
    configuration_ = new ui::ConfigurationEditor;
    editor_ = configuration_->target;
    if (!test_mode()) {
      QSettings settings;
      auto saved=QJsonDocument::fromJson(settings.value("closed_loop/configuration").toByteArray());
      try {
        if (saved.isObject()) {
          configuration_->load(saved.object());
          configuration_path_=settings.value("closed_loop/path").toString();
        } else {
          editor_->settings(false);
          configuration_->load({{"closed_loop_version",1},{"scene",editor_->scenario()}});
        }
      } catch (const std::exception& e) { qWarning("Cannot restore configuration: %s",e.what()); }
    }
    auto *configuration = new QWidget;
    auto *configuration_layout = new QVBoxLayout(configuration);
    configuration_layout->setContentsMargins(6, 10, 6, 6);
    configuration_layout->addWidget(configuration_, 1);
    config_file_ = new QLabel;
    config_file_->setObjectName("configuration_file");
    config_file_->setWordWrap(true);
    configuration_layout->addWidget(config_file_);
    update_configuration_path();
    auto *actions = new QHBoxLayout;
    auto *load_config = new QPushButton("打开配置");
    load_config->setObjectName("open_config");
    auto *save = new QPushButton("保存");
    save->setObjectName("save_config");
    auto *save_as = new QPushButton("另存为");
    save_as->setObjectName("save_config_as");
    actions->addWidget(load_config); actions->addWidget(save); actions->addWidget(save_as);
    configuration_layout->addLayout(actions);
    apply_ = new QPushButton("应用并运行");
    apply_->setObjectName("apply_config");
    auto* run_actions=new QHBoxLayout;run_actions->addWidget(apply_,1);
    stop_=new QPushButton("停止");stop_->setObjectName("stop_run");stop_->setEnabled(false);run_actions->addWidget(stop_);
    configuration_layout->addLayout(run_actions);
    connect(stop_,&QPushButton::clicked,this,[this]{restart_requested_=false;stop_live();});
    pages->addWidget(configuration);
    auto *inspection = new QWidget;
    auto *inspection_layout = new QVBoxLayout(inspection);
    pages->addWidget(inspection);
    configuration_->section->addTab("监测");
    side->addWidget(configuration_->section);
    connect(configuration_->section,&QTabBar::currentChanged,pages,[pages](int index){
      pages->setCurrentIndex(index==4 ? 1 : 0);
    });
    side->addWidget(pages, 1);
    side = inspection_layout;
    connect(apply_, &QPushButton::clicked, this, [this] { run_configuration(); });
    connect(load_config, &QPushButton::clicked, this, [this] {
      const auto path=QFileDialog::getOpenFileName(this,"打开仿真配置",configuration_path_,"JSON (*.json)");
      if (!path.isEmpty()) load_configuration(path);
    });
    auto save_configuration=[this](bool choose_path) {
      auto path=configuration_path_;
      if (choose_path || path.isEmpty()) path=QFileDialog::getSaveFileName(this,"保存仿真配置",
        path.isEmpty() ? "closed_loop.json" : path,"JSON (*.json)");
      if (!path.isEmpty()) write_configuration(path,true);
    };
    connect(save,&QPushButton::clicked,this,[=]{save_configuration(false);});
    connect(save_as,&QPushButton::clicked,this,[=]{save_configuration(true);});
    auto *files = new QHBoxLayout;
    files->addWidget(run);
    side->addLayout(files);
    target_ = new QComboBox;
    side->addWidget(target_);
    auto *views = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
      auto *b = new QPushButton(QStringList{"透视", "俯视", "正视"}[i]);
      views->addWidget(b);
      connect(b, &QPushButton::clicked, this, [this, i] { scene_->view(i); });
    }
    side->addLayout(views);
    metrics_ = new QLabel;
    metrics_->setWordWrap(true);
    metrics_->setMinimumHeight(170);
    side->addWidget(metrics_);
    side->addWidget(new QLabel("命中事件"));
    hits_ = new QListWidget;
    hits_->setToolTip("当前运行的最近命中事件；板上小绿点表示接触位置，随板运动并保留 5 秒，有效性见列表");
    hits_->setMinimumHeight(110);
    hits_->setMaximumHeight(180);
    side->addWidget(hits_);
    auto *toggles = new QHBoxLayout;
    auto *trails = new QCheckBox("轨迹");
    trails->setChecked(true);
    auto *normals = new QCheckBox("法线");
    toggles->addWidget(trails);
    toggles->addWidget(normals);
    side->addLayout(toggles);
    side->addStretch();
    split->addWidget(sidebar);
    split->setStretchFactor(0, 1);
    layout->addWidget(split, 1);
    auto *controls = new QWidget;
    auto *cl = new QVBoxLayout(controls);
    cl->setContentsMargins(20, 14, 20, 14);
    auto *row = new QHBoxLayout;
    clock_ = new QLabel("实时仿真 · 等待启动");
    clock_->setMinimumWidth(180);row->addWidget(clock_,1);
    auto* live_note=new QLabel("实时画面 · 点击停止结束");row->addWidget(live_note);
    cl->addLayout(row);
    status_ = new QLabel;
    status_->setStyleSheet("color:#64748b;font-size:11px");status_->setWordWrap(true);
    status_->hide();
    cl->addWidget(status_);
    layout->addWidget(controls);
    setCentralWidget(root);
    // Explicit list views avoid platform-native popup styling overriding the white theme.
    for (auto *combo : findChildren<QComboBox *>()) {
      auto *view = new QListView(combo);
      view->setFrameShape(QFrame::NoFrame);
      view->setSpacing(2);
      view->setTextElideMode(Qt::ElideNone);
      combo->setView(view);
      combo->setCursor(Qt::PointingHandCursor);
      combo->setMaxVisibleItems(10);
    }
    setStyleSheet(R"(
      QWidget { background:#ffffff; color:#334155;
        font-family:'Noto Sans CJK SC',sans-serif; font-size:12px; }
      QPushButton { background:#ffffff; border:1px solid #dce2ea; border-radius:6px;
        padding:7px 12px; }
      QPushButton:hover { background:#f5f8fc; border-color:#b8c6d9; }
      QPushButton:pressed { background:#eaf0f8; }
      QPushButton:focus { border-color:#7aa5f5; }
      QPushButton:disabled { background:#f8fafc; color:#a8b2c1; border-color:#e8edf3; }
      QPushButton#play, QPushButton#apply_config { background:#2563eb; color:#ffffff;
        border-color:#2563eb; font-weight:500; }
      QPushButton#play:hover, QPushButton#apply_config:hover { background:#1d4ed8; }
      QPushButton#play:pressed, QPushButton#apply_config:pressed { background:#1e40af; }
      QPushButton#apply_config:disabled { background:#a9c1f4; border-color:#a9c1f4; }
      QLineEdit, QPlainTextEdit, QComboBox, QDoubleSpinBox, QSpinBox { background:#f8fafc; border:1px solid #e0e6ef;
        border-radius:6px; min-height:18px; padding:6px 10px; selection-background-color:#dbeafe;
        selection-color:#1e40af; }
      QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover { border-color:#b8c6d9; }
      QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus { background:#ffffff;
        border-color:#7aa5f5; }
      QComboBox:disabled, QDoubleSpinBox:disabled, QSpinBox:disabled { color:#a8b2c1;
        background:#f8fafc; border-color:#e8edf3; }
      QComboBox { padding-right:32px; }
      QComboBox::drop-down { subcontrol-origin:padding; subcontrol-position:top right;
        width:30px; border:0; }
      QComboBox::down-arrow { image:url(:/controls/chevron-down.svg); width:16px; height:16px; }
      QComboBox QAbstractItemView { background:#ffffff; border:1px solid #dce2ea;
        border-radius:6px; padding:4px; outline:0;
        selection-background-color:#edf3ff; selection-color:#1d4ed8; }
      QComboBox QAbstractItemView::item { min-height:22px; padding:5px 10px; border-radius:4px; }
      QComboBox QAbstractItemView::item:hover, QComboBox QAbstractItemView::item:selected {
        background:#edf3ff; color:#1d4ed8; }
      QDoubleSpinBox, QSpinBox { padding-right:64px; }
      QDoubleSpinBox::up-button, QSpinBox::up-button { subcontrol-origin:border;
        subcontrol-position:top right; margin:3px 3px 3px 0; width:26px; height:26px;
        border:0; border-radius:4px; background:#edf1f6; }
      QDoubleSpinBox::down-button, QSpinBox::down-button { subcontrol-origin:border;
        subcontrol-position:top right; margin:3px 32px 3px 0; width:26px; height:26px;
        border:0; border-radius:4px; background:#edf1f6; }
      QDoubleSpinBox::up-button:hover, QSpinBox::up-button:hover,
      QDoubleSpinBox::down-button:hover, QSpinBox::down-button:hover { background:#dce8fc; }
      QDoubleSpinBox::up-button:pressed, QSpinBox::up-button:pressed,
      QDoubleSpinBox::down-button:pressed, QSpinBox::down-button:pressed { background:#c8dafa; }
      QDoubleSpinBox::up-button:off, QSpinBox::up-button:off,
      QDoubleSpinBox::down-button:off, QSpinBox::down-button:off { background:#f8fafc; }
      QDoubleSpinBox::up-arrow, QSpinBox::up-arrow { image:url(:/controls/plus.svg);
        width:14px; height:14px; }
      QDoubleSpinBox::down-arrow, QSpinBox::down-arrow { image:url(:/controls/minus.svg);
        width:14px; height:14px; }
      QDoubleSpinBox::up-arrow:off, QSpinBox::up-arrow:off,
      QDoubleSpinBox::down-arrow:off, QSpinBox::down-arrow:off { image:none; }
      QTabWidget::pane { border:1px solid #e8edf3; border-radius:6px; }
      QTabBar::tab { padding:8px 10px; border-bottom:2px solid transparent; }
      QTabBar::tab:hover { color:#2563eb; }
      QTabBar::tab:selected { color:#2563eb; border-bottom:2px solid #2563eb; }
      QListWidget { background:#ffffff; border:1px solid #e2e8f0; border-radius:6px; }
      QListWidget::item { padding:10px; }
      QListWidget::item:selected { background:#edf3ff; color:#1e40af; }
      QSplitter::handle { background:#f1f5f9; }
      QCheckBox::indicator { width:13px; height:13px; border:1px solid #94a3b8;
        border-radius:3px; background:#ffffff; }
      QCheckBox::indicator:checked { background:#2563eb; border-color:#2563eb; }
      QScrollBar:vertical { width:6px; background:transparent; margin:0; }
      QScrollBar::handle:vertical { background:#cbd5e1; border-radius:3px; min-height:24px; }
      QScrollBar::handle:vertical:hover { background:#94a3b8; }
      QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
      QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background:transparent; }
    )");
    connect(run, &QPushButton::clicked, this, [this] { run_scenario(); });
    connect(target_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int i) {
      scene_->target = i - 1;
      scene_->distance = scene_->target_distance();
      refresh();
    });
    connect(trails, &QCheckBox::toggled, this, [this](bool on) {
      scene_->trails = on;
      scene_->update();
    });
    connect(normals, &QCheckBox::toggled, this, [this](bool on) {
      scene_->normals = on;
      scene_->update();
    });
  }
  ~Window() override
  {
    if (process_ && process_->state() != QProcess::NotRunning) {
      process_->disconnect(this);
      process_->terminate();
      if (!process_->waitForFinished(500)) {
        process_->kill();
        process_->waitForFinished(500);
      }
    }
  }
  bool load(const QString &path, bool dialog = true)
  {
    try {
      auto next = load_trace(path);
      pause();
      scene_->trace = nullptr;
      trace_ = std::move(next);
      scene_->hide_hit_statistics();
      camera_->setVisible(!trace_.camera.empty());
      frame_ = 0;
      scene_->frame = 0;
      scene_->trace = &trace_;
      target_->clear();
      target_->addItem("全部目标");
      for (const auto &t : trace_.frames.front().targets)
        target_->addItem(label(t.kind) + " · " + q(t.id));

      hits_->clear();
      for (const auto &h : trace_.hits) {
        auto *item = new QListWidgetItem(number(h.time, 3) + " s · " + q(h.target_id) + " / " +
                                           QString::number(h.plate_id) + "\n" +
                                           (h.effective ? "有效命中" : q(h.reason)),
                                         hits_);
        item->setData(Qt::UserRole, h.time);
      }
      setWindowTitle(trace_.name);
      status_->hide();
      seek(int(trace_.frames.size()-1));
      return true;
    } catch (const std::exception &e) {
      if (dialog)
        QMessageBox::critical(this, "载入失败", e.what());
      else
        qWarning("%s", e.what());
      return false;
    }
  }
  bool select_target(const QString &id)
  {
    if (trace_.frames.empty()) return false;
    const auto &targets = trace_.frames.front().targets;
    for (size_t i = 0; i < targets.size(); ++i)
      if (q(targets[i].id) == id) { target_->setCurrentIndex(static_cast<int>(i)+1); return true; }
    return false;
  }
  bool busy() const { return process_ != nullptr; }
  static bool test_mode()
  {
    const auto args = QCoreApplication::arguments();
    return args.contains("--self-test") || args.contains("--self-test-live") || args.contains("--self-test-config") || args.contains("--self-test-loop-config");
  }
  void run_configuration()
  {
    if(process_){restart_requested_=true;stop_live();return;}
    if (!prepare_run()) return;
    const auto input = temporary_->filePath("scenario.json");
    if (!write_configuration(input)) return;
    run_process(input, true);
  }
  static void validate_json_precision(QJsonValue value) {
    if (value.isDouble() && std::abs(value.toDouble())>9007199254740991.)
      throw std::runtime_error("界面不支持超过 2^53−1 的数值；请使用 CLI 运行该配置，避免精度丢失");
    if (value.isArray()) for (const auto& item:value.toArray()) validate_json_precision(item);
    if (value.isObject()) { auto object=value.toObject(); for(auto it=object.begin();it!=object.end();++it) validate_json_precision(it.value()); }
  }
  bool load_configuration(const QString& path,bool dialog=true) {
    try {
      QFile file(path);
      if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("无法打开配置文件");
      if (file.size()>16*1024*1024) throw std::runtime_error("配置文件超过 16 MiB");
      QJsonParseError parse_error;
      auto doc=QJsonDocument::fromJson(file.readAll(),&parse_error);
      if (parse_error.error!=QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error("配置 JSON 格式错误");
      auto root=doc.object(); validate_json_precision(root);
      if (!root.contains("closed_loop_version")) root={{"closed_loop_version",1},{"scene",root},{"algorithm",QJsonObject{{"kind","none"}}}};
      QString error;
      if (!validate_configuration(root,error)) throw std::runtime_error(error.toStdString());
      // Prepare a detached editor first, so an unsupported value cannot partially replace the current form.
      auto candidate=std::make_unique<ui::ConfigurationEditor>(); candidate->load(root);
      configuration_->load(root);
      configuration_path_=QFileInfo(path).absoluteFilePath(); update_configuration_path();
      persist_configuration(configuration_->configuration());
      status_->setText("已载入："+configuration_path_); status_->show(); return true;
    } catch (const std::exception& e) { configuration_error(QString::fromUtf8(e.what()),dialog); return false; }
  }
  bool configuration_test()
  {
    configuration_->findChild<QComboBox*>("sensor.output")->setCurrentIndex(1);
    editor_->duration->setValue(1);
    editor_->shoot->setChecked(true);
    for (int test_case = 0; test_case < 10; ++test_case) {
      const int k = test_case < 4 ? test_case : 0;
      editor_->kind->setCurrentIndex(k);
      if (k == 0) {
        editor_->mx->kind->setCurrentIndex(1);
        editor_->mx->velocity->setValue(.4);
        editor_->my->kind->setCurrentIndex(5);
        editor_->rotation->kind->setCurrentIndex(3);
        editor_->rotation->velocity->setValue(2);
        editor_->armor->setCurrentIndex(1);
        editor_->marking->setCurrentIndex(test_case%editor_->marking->count());
      }
      if (test_case >= 4) {
        editor_->mx->kind->setCurrentIndex(test_case - 4);
        editor_->my->kind->setCurrentIndex(0);
        editor_->rotation->kind->setCurrentIndex(0);
      }
      apply_->click();
      if (!process_) return false;
      QEventLoop loop;
      QTimer timeout; timeout.setSingleShot(true);
      connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
      connect(process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), &loop,
              [&loop](int, QProcess::ExitStatus) { loop.quit(); });
      timeout.start(10000);
      loop.exec();
      if (busy() || !last_run_ok_ || trace_.frames.empty() || trace_.frames.back().time != 1) return false;
      const auto &t = trace_.frames.front().targets.front();
      if (static_cast<int>(t.kind) != k || t.id != "target" || scene_->target != 0) return false;
      if(k==0 && t.armor_marking!=editor_->marking->currentData().toString().toStdString())return false;
      if(k==1 && t.armor_marking!="outpost")return false;
      if (test_case == 0 && (std::abs(t.velocity.x - .4) > 1e-8 ||
                             std::abs(t.surfaces.front().width - .230) > 1e-8)) return false;
      if (k >= 2 && std::abs(t.position.z - 2.6) > 1e-8) return false;
      if (test_case == 4 && std::none_of(trace_.hits.begin(), trace_.hits.end(),
                                       [](const Hit &h) { return h.effective; })) return false;
    }
    return true;
  }
  bool loop_configuration_test() {
    auto check=[](bool ok,const char* message){ if(!ok) throw std::runtime_error(message); };
    auto read=[](const QString& path){ QFile f(path); if(!f.open(QIODevice::ReadOnly)) throw std::runtime_error("test file missing"); return f.readAll(); };
    auto put=[](const QString& path,QJsonObject root){ QFile f(path); if(!f.open(QIODevice::WriteOnly)) throw std::runtime_error("test file write failed"); f.write(QJsonDocument(root).toJson()); };
    auto number=[&](QString key,double value){
      auto* field=configuration_->findChild<QDoubleSpinBox*>(key);
      check(field!=nullptr,"missing numeric control"); field->setValue(value);
    };
    try {
      QTemporaryDir files; check(files.isValid(),"test directory missing");
      configuration_->findChild<QComboBox*>("sensor.output")->setCurrentIndex(1);
      const QString saved=files.filePath("loop.json"), imported=files.filePath("imported.json"), bad=files.filePath("invalid.json");
      editor_->duration->setValue(1); editor_->shoot->setChecked(false);
      number("sensor.calibration.fx",1111.125); number("sensor.calibration.fy",1122.25);
      number("sensor.calibration.width",799); number("sensor.calibration.height",600);
      configuration_->findChild<QDoubleSpinBox*>("sensor.calibration.width")->stepUp();
      check(configuration_->findChild<QDoubleSpinBox*>("sensor.calibration.width")->value()==800,"Image size button step failed");
      number("sensor.calibration.cx",400); number("sensor.calibration.cy",300);
      number("sensor.frequency",144); number("sensor.latency",12); number("sensor.pixel_sigma",.3);
      number("actuator.initial_yaw",30); number("actuator.command_latency",4);
      number("actuator.max_speed",2); number("actuator.feedback_latency",6);
      number("scene.world.gravity.2",0); number("runner.algorithm_frequency",250); number("runner.log_frequency",40);
      configuration_->findChild<QCheckBox*>("actuator.ideal")->setChecked(false);
      configuration_->findChild<QComboBox*>("algorithm.kind")->setCurrentIndex(0);
      check(write_configuration(saved,true),"UI save failed");
      const auto expected=QJsonDocument::fromJson(read(saved)).object();
      check(expected["sensor"].toObject()["latency"].toDouble()==.012,"Milliseconds conversion failed");
      check(std::abs(expected["actuator"].toObject()["initial_yaw"].toDouble()-pi/6)<1e-9,"Angle conversion failed");
      number("sensor.calibration.fx",900); number("actuator.max_speed",6);
      check(load_configuration(saved,false),"UI reload failed");
      check(configuration_->configuration()==expected,"Save/reload changed configuration");
      check(configuration_->findChild<QDoubleSpinBox*>("sensor.calibration.fx")->value()==1111.125,"Intrinsics were not restored");
      check(configuration_->findChild<QDoubleSpinBox*>("actuator.max_speed")->value()==2,"Actuator value was not restored");
      // Arbitrary extra targets, high-precision values, features and events survive ordinary edits.
      auto root=expected, scene=root["scene"].toObject(); auto targets=scene["targets"].toArray();
      auto first=targets[0].toObject(); first["id"]="primary"; first["tilt"]=.212345678901234;
      first["rotation"]=QJsonObject{{"kind","stationary"},{"initial",.123456789012345}};
      targets[0]=first;
      targets.append(QJsonObject{{"id","outpost"},{"kind","outpost"},{"position",QJsonArray{7,0,.3}}});
      scene["targets"]=targets;
      scene["events"]=QJsonArray{QJsonObject{{"time",.5},{"type","stop_outpost"},{"target_id","outpost"}}};
      scene["shots"]=QJsonArray{QJsonObject{{"time",.2},{"id","preset"},{"origin",QJsonArray{0,0,.3}},{"velocity",QJsonArray{23,0,0}}}};
      root["scene"]=scene;
      root=ui::set_path(root,{"sensor","calibration","fx"},1111.123456789012).toObject();
      root=ui::set_path(root,{"sensor","appearances"},QJsonObject{{"primary",QJsonObject{
        {"model_id","custom-v1"},{"measured_class","armor"},{"points",QJsonArray{QJsonArray{0,0,0}}}}}}).toObject();
      root["commands"]=QJsonArray{QJsonObject{{"time",.1},{"yaw",0},{"pitch",0},{"fire",true},{"request_id","test-shot"}}};
      put(imported,root); check(load_configuration(imported,false),"Advanced configuration import failed");
      auto reloaded=configuration_->configuration();
      check(reloaded["scene"]==root["scene"],"Imported scene data lost");
      check(ui::at_path(reloaded,{"sensor","calibration","fx"})==ui::at_path(root,{"sensor","calibration","fx"}),"Intrinsics precision lost");
      check(reloaded["commands"]==root["commands"],"Prequeued commands lost");
      number("sensor.pixel_sigma",.8); editor_->px->setValue(4.5);
      auto edited=configuration_->configuration();
      check(edited["scene"].toObject()["targets"].toArray()[1]==targets[1],"Other target changed");
      check(edited["scene"].toObject()["targets"].toArray()[0].toObject()["tilt"]==first["tilt"],"Advanced target value lost");
      check(edited["scene"].toObject()["targets"].toArray()[0].toObject()["position"].toArray()[0].toDouble()==4.5,"Target form edit missing");
      check(write_configuration(saved,true),"Edited configuration save failed");
      auto good_bytes=read(saved);
      number("sensor.frequency",1001);
      check(!write_configuration(saved,true),"Invalid sampling rate was saved");
      check(read(saved)==good_bytes,"Invalid save overwrote existing file");
      number("sensor.frequency",144);
      auto before=configuration_->configuration();
      put(bad,ui::set_path(root,{"actuator","pitch_min"},2).toObject());
      check(!load_configuration(bad,false),"Invalid joint limits imported");
      check(configuration_->configuration()==before,"Failed import modified form");
      run_configuration(); check(process_!=nullptr,"Closed-loop process did not start");
      QEventLoop loop; QTimer timeout; timeout.setSingleShot(true);
      connect(&timeout,&QTimer::timeout,&loop,&QEventLoop::quit);
      connect(process_,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),&loop,[&](int,QProcess::ExitStatus){loop.quit();});
      timeout.start(10000); loop.exec(); check(!busy() && last_run_ok_,"Closed-loop UI run failed");
      auto output=QJsonDocument::fromJson(read(temporary_->filePath("trace.json"))).object();
      check(output["engine"].toString()=="rm_sim_loop/1","Wrong backend selected");
      check(output["resolved"].toObject()["sensor"].toObject()["pixel_sigma"].toDouble()==.8,"Sensor edit did not reach backend");
      check(output["resolved"].toObject()["actuator"].toObject()["max_speed"].toDouble()==2,"Actuator edit did not reach backend");
      check(output["resolved"].toObject()["algorithm_frequency"].toDouble()==250,"Scheduler edit did not reach backend");
      check(output["launches"].toArray().size()==2,"Imported firing events were not executed");
      // New image controls survive file roundtrip, including millimetre conversion.
      configuration_->findChild<QComboBox*>("sensor.output")->setCurrentIndex(0);
      configuration_->findChild<QComboBox*>("sensor.image.color")->setCurrentIndex(1);
      number("sensor.image.light_length",60);number("sensor.image.light_width",8);
      number("sensor.image.noise_sigma",1.25);
      check(write_configuration(saved,true),"Camera image configuration save failed");
      check(load_configuration(saved,false),"Camera image configuration reload failed");
      auto camera=configuration_->configuration()["sensor"].toObject();
      check(camera["output"].toString()=="image" && camera["image"].toObject()["color"].toString()=="red","Camera output/color lost");
      check(std::abs(camera["image"].toObject()["light_length"].toDouble()-.060)<1e-9,"Camera millimetres conversion failed");
      check(camera["image"].toObject()["noise_sigma"].toDouble()==1.25,"Camera noise lost");
      auto* controller=configuration_->findChild<QComboBox*>("algorithm.kind");
      auto* adapter=configuration_->findChild<QComboBox*>("algorithm.adapter");
      check(controller && adapter,"Missing external adapter controls");
      controller->setCurrentIndex(controller->findData("external"));
      adapter->setCurrentIndex(adapter->findData("sp_vision"));
      number("algorithm.port",17002);number("algorithm.connect_timeout",2500);number("algorithm.step_timeout",800);
      check(write_configuration(saved,true),"External protocol configuration save failed");
      check(load_configuration(saved,false),"External protocol configuration reload failed");
      auto external=configuration_->configuration()["algorithm"].toObject();
      check(external["kind"].toString()=="external" && external["adapter"].toString()=="sp_vision","External selection lost");
      check(external["connect_timeout"].toDouble()==2.5 && external["step_timeout"].toDouble()==.8,"External timeout units lost");
      check(!external.contains("gain") && !external.contains("fire"),"Unrelated controller fields leaked into adapter config");
      adapter->setCurrentIndex(adapter->findData("generic"));
      check(write_configuration(saved,true) && load_configuration(saved,false),"Generic adapter switch failed");
      check(configuration_->configuration()["algorithm"].toObject()["adapter"].toString()=="generic","Generic adapter selection lost");
      controller->setCurrentIndex(controller->findData("none"));
      check(configuration_->configuration()["algorithm"].toObject().size()==1,"External fields leaked into disabled algorithm");
      controller->setCurrentIndex(controller->findData("external"));
      configuration_->section->setCurrentIndex(3);configuration_->scheduler_tabs->setCurrentIndex(1);
      status_->hide();
      qInfo("Closed-loop editor: save/reload, preservation, validation and execution passed");
      return true;
    } catch (const std::exception& e) { qWarning("Closed-loop editor test failed: %s",e.what()); return false; }
  }
  bool self_test()
  {
    if(trace_.frames.empty())return false;
    if(live_mode_ && !last_run_ok_)return false;
    if(!findChildren<QSlider*>().isEmpty() || findChild<QPushButton*>("play") ||
       findChild<QPushButton*>("step") || findChild<QPushButton*>("reset"))return false;
    if(!trace_.camera.empty()){seek(int(trace_.frames.size()-1));if(!camera_->has_image())return false;}
    target_->setCurrentIndex(1);
    if(scene_->target!=0)return false;
    scene_->view(1);scene_->view(2);scene_->view(0);return true;
  }
  double live_time() const {return trace_.frames.empty()?0:trace_.frames.back().time;}
  uint64_t live_frames() const {return live_frames_;}
  uint64_t live_sessions() const {return live_sessions_;}
  uint64_t live_commands() const {return live_statistics_["commands"].toVariant().toULongLong();}
  bool live_camera() const {return camera_->has_image();}


private:
  ui::ScenarioEditor *editor_;
  ui::ConfigurationEditor *configuration_;
  QLabel *config_file_;
  QString configuration_path_;
  bool last_run_ok_=false;
  QPushButton *apply_,*stop_;
  bool run_cancelled_=false;
  QByteArray process_output_;
  Trace trace_;
  Scene *scene_;
  CameraView* camera_;
  QComboBox *target_;
  QLabel *metrics_, *clock_, *status_;
  QString external_status_;
  QListWidget *hits_;
  QByteArray error_output_;
  QJsonObject live_statistics_;
  bool live_mode_=false,restart_requested_=false;
  uint64_t live_frames_=0;
  uint64_t live_sessions_=0;

  int frame_ = 0;

  QProcess *process_ = nullptr;
  std::unique_ptr<QTemporaryDir> temporary_;
  void pause()
  {

  }
  void seek(int i)
  {
    if (trace_.frames.empty()) return;
    frame_ = std::clamp(i, 0, static_cast<int>(trace_.frames.size() - 1));

    refresh();
  }
  void refresh()
  {
    if (trace_.frames.empty()) return;
    const auto &s = trace_.frames[frame_];
    if(live_mode_){
      const auto recent_hits=std::count_if(trace_.hits.begin(),trace_.hits.end(),[&](const Hit& hit){
        return hit.effective && hit.time>s.time-5. && hit.time<=s.time;
      });
      scene_->hit_statistics(live_statistics_["shots"].toVariant().toULongLong(),
        live_statistics_["effective_hits"].toVariant().toULongLong(),recent_hits*20./5.);
    }
    scene_->frame = frame_;
    scene_->update();
    if(!live_mode_){
      auto image=std::upper_bound(trace_.camera.begin(),trace_.camera.end(),s.time,
        [](double time,const CameraFrame& frame){return time<frame.ready;});
      camera_->display(image==trace_.camera.begin()?nullptr:&*std::prev(image));
    }
    clock_->setText(QString(live_mode_?"实时 · %1 s":"当前状态 · %1 s").arg(s.time,0,'f',2));
    QString text;
    if (scene_->target >= 0 && scene_->target < static_cast<int>(s.targets.size())) {
      const auto &t = s.targets[scene_->target];
      text = label(t.kind) + "\n\n位置 / m　" + number(t.position.x) + ", " + number(t.position.y) +
             ", " + number(t.position.z) + "\n速度 / m·s⁻¹　" + number(norm(t.velocity)) +
             "\n加速度 / m·s⁻²　" + number(norm(t.acceleration)) + "\n角速度 / rad·s⁻¹　" +
             number(t.rotation.velocity) + "\n角加速度 / rad·s⁻²　" +
             number(t.rotation.acceleration) + "\n激活状态　" + q(t.activation) + "\n进度 / 组　" +
             QString::number(t.completed_groups);
    } else {
      int contacts = 0, accepted = 0;
      for (const auto &h : trace_.hits)
        if (h.time <= s.time) {
          ++contacts;
          accepted += h.effective;
        }
      if(live_mode_){contacts=live_statistics_["contacts"].toInt();accepted=live_statistics_["effective_hits"].toInt();}
      text = "目标数　" + QString::number(s.targets.size()) +
             "\n飞行中弹丸　" + QString::number(s.projectiles.size()) + "\n累计接触　" +
             QString::number(contacts) + "\n有效命中　" + QString::number(accepted);
    }
    metrics_->setText(text);
  }
  void update_configuration_path() {
    config_file_->setText(configuration_path_.isEmpty() ? "尚未保存配置文件" : QFileInfo(configuration_path_).fileName());
    config_file_->setToolTip(configuration_path_);
  }
  void configuration_error(const QString& message,bool dialog=true) {
    status_->setText(message); status_->show();
    if (test_mode() || !dialog) qWarning("%s",qPrintable(message));
    else QMessageBox::warning(this,"配置",message);
  }
  bool validate_configuration(const QJsonObject& root,QString& error) {
    QTemporaryFile file;
    const auto bytes=QJsonDocument(root).toJson();
    if (!file.open() || file.write(bytes)!=bytes.size() || !file.flush()) {
      error="无法创建配置校验文件"; return false;
    }
    QProcess validator;
    validator.start(QCoreApplication::applicationDirPath()+"/rm_sim_loop",{"--validate",file.fileName()});
    if (!validator.waitForStarted(3000)) { error="找不到 rm_sim_loop；请同时构建 CLI"; return false; }
    if (!validator.waitForFinished(10000)) {
      validator.kill(); validator.waitForFinished(1000); error="配置校验超时"; return false;
    }
    if (validator.exitStatus()!=QProcess::NormalExit || validator.exitCode()!=0) {
      error="配置无效："+QString::fromUtf8(validator.readAllStandardError()).trimmed(); return false;
    }
    return true;
  }
  void persist_configuration(const QJsonObject& root) {
    if (test_mode()) return;
    QSettings settings;
    settings.setValue("closed_loop/configuration",QJsonDocument(root).toJson(QJsonDocument::Compact));
    settings.setValue("closed_loop/path",configuration_path_);
  }
  bool write_configuration(const QString &path,bool remember=false) {
    try {
      const auto root=configuration_->configuration();
      QString error;
      if (!validate_configuration(root,error)) { configuration_error(error); return false; }
      QSaveFile file(path); const auto bytes=QJsonDocument(root).toJson();
      if (!file.open(QIODevice::WriteOnly) || file.write(bytes)!=bytes.size() || !file.commit()) {
        configuration_error("配置保存失败："+file.errorString()); return false;
      }
      if (remember) {
        configuration_path_=QFileInfo(path).absoluteFilePath(); update_configuration_path();
        status_->setText("已保存："+configuration_path_); status_->show();
      }
      persist_configuration(root);
      return true;
    } catch (const std::exception& e) { configuration_error(QString::fromUtf8(e.what())); return false; }
  }
  bool prepare_run()
  {
    if (process_) return false;
    temporary_ = std::make_unique<QTemporaryDir>();
    if (!temporary_->isValid()) {
      status_->setText("无法创建临时目录");
      status_->show();
      return false;
    }
    return true;
  }
  void run_scenario()
  {
    const auto path = QFileDialog::getOpenFileName(this, "运行 JSON 场景", {}, "JSON (*.json)");
    if(!path.isEmpty() && load_configuration(path))run_configuration();
  }
  void stop_live() {
    if(!process_)return;
    run_cancelled_=true;status_->setText("正在停止实时仿真…");status_->show();
    process_->terminate();
    QPointer<QProcess> process=process_;
    QTimer::singleShot(3000,this,[process]{if(process && process->state()!=QProcess::NotRunning)process->kill();});
  }
  void receive_live() {
    for(;;){
      if(process_output_.size()<12)return;
      auto* prefix=reinterpret_cast<const uchar*>(process_output_.constData());
      auto magic=qFromBigEndian<quint32>(prefix),header_size=qFromBigEndian<quint32>(prefix+4),pixels=qFromBigEndian<quint32>(prefix+8);
      if(magic!=0x524d4c31 || header_size==0 || header_size>1024*1024 || pixels>64*1024*1024)
        throw std::runtime_error("实时画面数据头无效");
      auto total=12+quint64(header_size)+pixels;
      if(quint64(process_output_.size())<total)return;
      QJsonParseError error;
      auto header=QJsonDocument::fromJson(process_output_.mid(12,header_size),&error).object();
      if(error.error!=QJsonParseError::NoError || header["protocol"]!="rm-live/1" || header["type"]!="frame")
        throw std::runtime_error("实时画面协议无效");
      auto snapshot=parse_snapshot(header["frame"].toObject());
      if(!trace_.frames.empty() && snapshot.time<=trace_.frames.back().time){
        process_output_.remove(0,int(total));continue;
      }
      bool first=trace_.frames.empty();trace_.frames.push_back(std::move(snapshot));
      if(trace_.frames.size()>90)trace_.frames.erase(trace_.frames.begin());
      frame_=int(trace_.frames.size()-1);scene_->frame=frame_;scene_->trace=&trace_;
      if(first){
        target_->clear();target_->addItem("全部目标");
        for(const auto& target:trace_.frames.back().targets)target_->addItem(label(target.kind)+" · "+q(target.id));
        if(!trace_.frames.back().targets.empty())target_->setCurrentIndex(1);
      }
      if(header.contains("camera")){
        auto camera=header["camera"].toObject();int w=camera["width"].toInt(),h=camera["height"].toInt();
        if(w<1 || h<1 || quint64(w)*h>16777216 || quint64(w)*h*3!=pixels ||
           camera["stride"].toInt()!=w*3 || camera["encoding"]!="bgr8")
          throw std::runtime_error("实时相机像素范围无效");
        camera_->display_live(process_output_.mid(12+header_size,pixels),w,h,camera["capture_time"].toDouble(),camera["ready_time"].toDouble(),camera["renderer"].toString("cpu").toUpper());
        camera_->setToolTip(QString("相机渲染：%1\n设备：%2\n原生分辨率 %3 × %4")
          .arg(camera["renderer"].toString("cpu").toUpper(),camera["device"].toString("CPU")).arg(w).arg(h));
        camera_->show();
      }else if(pixels)throw std::runtime_error("缺少实时相机元数据");
      live_statistics_=header["statistics"].toObject();
      trace_.hits.clear();hits_->clear();
      for(const auto& value:header["hits"].toArray()){
        auto hit=parse_hit(value.toObject());trace_.hits.push_back(hit);
        if(hits_->count()==128)delete hits_->takeItem(0);
        hits_->addItem(number(hit.time,3)+" s · "+q(hit.target_id)+" / "+QString::number(hit.plate_id)+" · "+(hit.effective?"有效命中":q(hit.reason)));
      }
      auto adapter=header["adapter"].toObject(),info=adapter["telemetry"].toObject();
      const double wall=header["wall_time"].toDouble();
      const auto camera_status=QString("相机 %1 Hz / 设定 %2 Hz｜%4｜渲染丢弃 %3")
        .arg(wall>0?live_statistics_["camera_frames"].toDouble()/wall:0,0,'f',1)
        .arg(live_statistics_["camera_frequency"].toDouble(),0,'f',0)
        .arg(live_statistics_["dropped_camera_jobs"].toVariant().toULongLong())
        .arg(header["camera"].isObject()?header["camera"].toObject()["renderer"].toString("cpu").toUpper():"无图像");
      if(!adapter.isEmpty() && adapter["state"]!="connected"){
        status_->setText("等待自瞄连接｜仿真继续运行｜"+camera_status);
      }else if(!adapter.isEmpty()){
        status_->setText(QString("自瞄已连接｜图像 %1 / 反馈 %2｜目标 %3｜控制 %4｜yaw %5° / pitch %6°")
          .arg(info["images_received"].toVariant().toULongLong()).arg(info["feedback_received"].toVariant().toULongLong())
          .arg(info["targets"].toInt()).arg(info["control"].toBool()?"开":"关")
          .arg(info["yaw"].toDouble()*180/pi,0,'f',2).arg(info["pitch"].toDouble()*180/pi,0,'f',2)+"｜"+camera_status+
          QString("｜待发丢弃 %1").arg(adapter["dropped_input_images"].toVariant().toULongLong()));
      }else status_->setText("实时仿真运行中｜"+camera_status);
      status_->show();++live_frames_;last_run_ok_=true;refresh();
      process_output_.remove(0,int(total));
    }
  }
  void run_process(const QString &path, bool configured)
  {
    Q_UNUSED(configured);
    const auto args=QCoreApplication::arguments();
    live_mode_=!args.contains("--self-test-config") && !args.contains("--self-test-loop-config");
    apply_->setEnabled(live_mode_);apply_->setText(live_mode_?"应用并重启":"应用并运行");
    stop_->setEnabled(true);run_cancelled_=false;process_output_.clear();error_output_.clear();
    last_run_ok_=false;external_status_.clear();
    const auto output = temporary_->filePath("trace.json");
    QFile input(path);bool closed_loop=false;QJsonObject configuration;
    if(input.open(QIODevice::ReadOnly)){configuration=QJsonDocument::fromJson(input.readAll()).object();closed_loop=configuration.contains("closed_loop_version");}
    if(live_mode_ && !closed_loop){
      configuration={{"closed_loop_version",1},{"scene",configuration},{"algorithm",QJsonObject{{"kind","none"}}}};
      auto wrapped=temporary_->filePath("live.json");QFile file(wrapped);file.open(QIODevice::WriteOnly);file.write(QJsonDocument(configuration).toJson());file.close();
      run_backend(wrapped,output,true,configuration);return;
    }
    run_backend(path,output,closed_loop,configuration);
  }
  void run_backend(const QString& path,const QString& output,bool closed_loop,const QJsonObject& configuration){
    if(live_mode_){
      ++live_sessions_;
      scene_->trace=nullptr;trace_=Trace{};frame_=0;live_frames_=0;live_statistics_={};
      scene_->hit_statistics(0,0,0);
      camera_->display(nullptr);camera_->hide();hits_->clear();
      setWindowTitle(configuration["scene"].toObject()["name"].toString("仿真")+" · 实时");
    }
    status_->setText(live_mode_?"正在启动实时仿真 / 连接自瞄…":"正在计算测试场景…");status_->show();
    process_=new QProcess(this);
    process_->setProgram(QCoreApplication::applicationDirPath()+(closed_loop?"/rm_sim_loop":"/rm_sim"));
    QStringList arguments=live_mode_?QStringList{"--live",path}:QStringList{path,output};
    if(live_mode_ && (test_mode() || QCoreApplication::arguments().contains("--screenshot"))){
      double limit=QCoreApplication::arguments().contains("--self-test-live")?2.:configuration["scene"].toObject()["duration"].toDouble(1);
      arguments<<"--live-for="+QString::number(limit,'g',12);
    }
    process_->setArguments(arguments);
    connect(process_,&QProcess::readyReadStandardOutput,this,[this]{
      process_output_+=process_->readAllStandardOutput();
      if(live_mode_){try{receive_live();}catch(const std::exception& error){
        error_output_+=error.what();run_cancelled_=false;process_->kill();}}
      else process_output_.clear();
    });
    connect(process_,&QProcess::readyReadStandardError,this,[this]{
      error_output_+=process_->readAllStandardError();if(error_output_.size()>65536)error_output_=error_output_.right(65536);
    });
    connect(process_,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,[this,output](int code,QProcess::ExitStatus exit){
      error_output_+=process_->readAllStandardError();
      process_->deleteLater();process_=nullptr;apply_->setEnabled(true);apply_->setText("启动实时仿真");stop_->setEnabled(false);
      if(run_cancelled_){status_->setText("实时仿真已停止");status_->show();}
      else if(code==0 && exit==QProcess::NormalExit){
        if(!live_mode_){
          last_run_ok_=load(output,!test_mode());
          if(last_run_ok_ && !trace_.frames.front().targets.empty())select_target(q(trace_.frames.front().targets.front().id));
        }
        else {status_->setText("实时仿真已停止");status_->show();}
      }else{
        last_run_ok_=false;status_->setText("仿真连接或运行失败");status_->show();
        if(test_mode())qWarning("%s",error_output_.constData());
        else QMessageBox::critical(this,"仿真运行失败",QString::fromUtf8(error_output_));
      }
      if(live_mode_)clock_->setText(QString("已停止 · %1 s").arg(live_time(),0,'f',2));
      if(restart_requested_){restart_requested_=false;QTimer::singleShot(0,this,[this]{run_configuration();});}
    });
    connect(process_,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){
      if(error==QProcess::FailedToStart){status_->setText("找不到仿真程序，请重新构建 CLI");status_->show();
        process_->deleteLater();process_=nullptr;apply_->setEnabled(true);stop_->setEnabled(false);}
    });
    process_->start();
  }

};
}  // namespace

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("SPVision");
  QCoreApplication::setApplicationName("RMSimulation");
  Window window;
  QString path, screenshot, target_id, config_path, config_section;
  bool test = false, config_test = false, loop_config_test=false,live_test=false,restart_test=false;
  const auto args = app.arguments();
  for (int i = 1; i < args.size(); ++i) {
    if (args[i] == "--self-test")
      test = true;
    else if(args[i]=="--self-test-live")
      live_test=test=true;
    else if(args[i]=="--self-test-live-restart")
      restart_test=true;
    else if (args[i] == "--self-test-loop-config")
      loop_config_test = true;
    else if (args[i] == "--self-test-config")
      config_test = true;
    else if (args[i] == "--config" && i + 1 < args.size())
      config_path = args[++i];
    else if (args[i] == "--config-section" && i + 1 < args.size())
      config_section = args[++i];
    else if (args[i] == "--target" && i + 1 < args.size())
      target_id = args[++i];
    else if (args[i] == "--screenshot" && i + 1 < args.size())
      screenshot = args[++i];
    else if (args[i] == "--help") {
      qInfo("Usage: rm_sim_qt [trace.json] [--config configuration.json] [--config-section camera|actuator|scheduler|target] [--self-test] [--self-test-config] [--self-test-loop-config] [--target id] [--screenshot output.png]");
      return 0;
    } else if (path.isEmpty())
      path = args[i];
    else {
      qWarning("Unexpected argument");
      return 2;
    }
  }
  if (!config_path.isEmpty() && !window.load_configuration(config_path,false)) return 1;
  if (!config_section.isEmpty()) {
    auto* selector=window.findChild<QTabBar*>("configuration_section");
    int index=QStringList{"target","camera","actuator","scheduler"}.indexOf(config_section);
    if (index<0) return 2; selector->setCurrentIndex(index);
  }
  if (!path.isEmpty() && !window.load(path, false)) return 1;
  if (!target_id.isEmpty() && !window.select_target(target_id)) return 2;
  window.show();
  if (path.isEmpty() && !config_test && !loop_config_test) window.run_configuration();
  if (test || config_test || loop_config_test || !screenshot.isEmpty())
    QTimer::singleShot(150, &window, [&] {
      bool observed_live=false,restart_sent=false;
      if (window.busy()) {
        QEventLoop loop;
        QTimer poll, timeout;
        timeout.setSingleShot(true);
        QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
        QObject::connect(&poll, &QTimer::timeout, &loop, [&] {
          if(window.busy() && window.live_frames()>2 && window.live_camera())observed_live=true;
          if(restart_test && !restart_sent && window.live_time()>=.4 && window.live_camera()){
            restart_sent=true;window.run_configuration();
          }
          if (!window.busy() && (!restart_test || window.live_sessions()>=2)) loop.quit();
        });
        poll.start(20); timeout.start(30000); loop.exec();
      }
      bool ok = !window.busy() && (!test || window.self_test());
      if(live_test)ok=ok && observed_live && window.live_time()>=1.8 && window.live_camera();
      if(restart_test)ok=ok && restart_sent && window.live_sessions()==2;
      if(live_test)qInfo("Live UI: frames=%llu time=%.3f commands=%llu observed_while_running=%d",
        static_cast<unsigned long long>(window.live_frames()),window.live_time(),
        static_cast<unsigned long long>(window.live_commands()),int(observed_live));
      if (config_test) ok = window.configuration_test() && ok;
      if (loop_config_test) ok = window.loop_configuration_test() && ok;
      if (!target_id.isEmpty()) ok = window.select_target(target_id) && ok;
      app.processEvents();
      const bool saved = screenshot.isEmpty() || window.grab().save(screenshot);
      qInfo("Qt UI test: %s; screenshot: %s", ok ? "passed" : "failed", saved ? "saved" : "failed");
      app.exit(ok && saved ? 0 : 1);
    });
  return app.exec();
}
