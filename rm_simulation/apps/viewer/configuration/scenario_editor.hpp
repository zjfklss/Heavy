#pragma once
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QSettings>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <cmath>

namespace rm_sim::ui {
class CompactDoubleSpinBox : public QDoubleSpinBox {
 protected:
  QString textFromValue(double v) const override {
    QString text=QDoubleSpinBox::textFromValue(v);
    const auto separator=locale().decimalPoint();
    if (text.contains(separator)) {
      while (text.endsWith('0')) text.chop(1);
      if (text.endsWith(separator)) text.chop(1);
    }
    return text;
  }
};
inline QDoubleSpinBox * value(QFormLayout * form, QString text, QString id, double initial,
                              double minimum, double maximum, int decimals = 2) {
  auto * box = new CompactDoubleSpinBox;
  box->setObjectName(id);
  box->setDecimals(decimals);
  box->setRange(minimum, maximum);
  box->setValue(initial);
  box->setSingleStep(decimals > 2 ? .01 : .1);
  box->setKeyboardTracking(false);
  form->addRow(text, box);
  return box;
}
inline void visible(QFormLayout * form, QWidget * field, bool show) {
  field->setVisible(show);
  if (auto * label = form->labelForField(field)) label->setVisible(show);
}
class MotionEditor : public QWidget {
 public:
  QComboBox * kind;
  QDoubleSpinBox *velocity, *acceleration, *jerk, *amplitude, *frequency;
  MotionEditor(QString prefix, bool angular = false) {
    auto * form = new QFormLayout(this);
    kind = new QComboBox;
    kind->setObjectName(prefix + "_kind");
    const QStringList names{"静止",       "匀速",    "匀加速", "变加速（恒定 jerk）",
                            "正弦加速度", "随机运动"};
    const QStringList codes{"stationary", "velocity",   "acceleration",
                            "jerk",       "sinusoidal", "random"};
    for (int i = 0; i < names.size(); ++i) kind->addItem(names[i], codes[i]);
    form->addRow("运动", kind);
    QString unit = angular ? "rad" : "m";
    velocity = value(form, "初速度 / " + unit + "·s⁻¹", prefix + "_v", 0, -20, 20);
    acceleration = value(form, "加速度 / " + unit + "·s⁻²", prefix + "_a", 0, -20, 20);
    jerk = value(form, "加加速度 / " + unit + "·s⁻³", prefix + "_j", 1, -20, 20);
    amplitude = value(form, "振幅", prefix + "_amp", 1, 0, 20);
    frequency = value(form, "角频率 / rad·s⁻¹", prefix + "_freq", 2, .01, 20);
    auto update = [=] {
      int k = kind->currentIndex();
      visible(form, velocity, k >= 1 && k <= 4);
      visible(form, acceleration, k >= 2 && k <= 4);
      visible(form, jerk, k == 3);
      visible(form, amplitude, k >= 4);
      visible(form, frequency, k >= 4);
      qobject_cast<QLabel *>(form->labelForField(amplitude))
          ->setText(k == 4 ? "加速度振幅 / " + unit + "·s⁻²" : "位移振幅 / " + unit);
    };
    connect(kind, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [update](int) { update(); });
    update();
  }
  QJsonObject json(int seed) const {
    QJsonObject m{{"kind", kind->currentData().toString()}};
    int k = kind->currentIndex();
    if (k >= 1 && k <= 4) m["velocity"] = velocity->value();
    if (k >= 2 && k <= 4) m["acceleration"] = acceleration->value();
    if (k == 3) m["jerk"] = jerk->value();
    if (k >= 4) {
      m["amplitude"] = amplitude->value();
      m["frequency"] = frequency->value();
    }
    if (k == 5) m["seed"] = seed;
    return m;
  }
};
class ScenarioEditor : public QWidget {
 public:
  QComboBox *kind, *armor, *marking, *direction, *car_motion;
  QDoubleSpinBox *px, *py, *pz, *duration, *radius, *second_radius, *height_difference, *phase;
  QDoubleSpinBox *stop_time, *rune_a, *rune_omega, *axis_yaw;
  QSpinBox *seed, *shot_count;
  MotionEditor *mx, *my, *rotation;
  QCheckBox *shoot, *active;
  QDoubleSpinBox *ox, *oy, *oz, *yaw, *pitch, *muzzle, *shot_start, *shot_interval;
  ScenarioEditor() {
    auto * layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    auto * form = new QFormLayout;
    kind = new QComboBox;
    kind->setObjectName("target_kind");
    kind->addItem("四装甲板车", "car");
    kind->addItem("前哨站", "outpost");
    kind->addItem("小能量机关", "small_rune");
    kind->addItem("大能量机关", "big_rune");
    form->addRow("目标", kind);
    px = value(form, "X / m", "px", 4, -50, 50);
    py = value(form, "Y / m", "py", 0, -50, 50);
    pz = value(form, "装甲中心高度 / m", "pz", .3, .1, 10);
    duration = value(form, "预置事件范围 / s", "duration", 12, .1, 120);
    seed = new QSpinBox;
    seed->setObjectName("seed");
    seed->setRange(0, 1000000);
    seed->setValue(27);
    form->addRow("随机种子", seed);
    layout->addLayout(form);
    auto * car = new QWidget;
    auto * cf = new QFormLayout(car);
    cf->setContentsMargins(0, 0, 0, 0);
    armor = new QComboBox;
    armor->setObjectName("armor");
    armor->addItems({"小装甲", "大装甲"});
    cf->addRow("装甲", armor);
    marking=new QComboBox;
    marking->setObjectName("armor_marking");
    marking->addItem("英雄 1","1");marking->addItem("工程 2","2");
    marking->addItem("步兵 3","3");marking->addItem("步兵 4","4");
    marking->addItem("哨兵（炮塔图案）","sentry");marking->setCurrentIndex(2);
    cf->addRow("标识（2026）",marking);
    radius = value(cf, "前后半径 / m", "radius", .25, .1, 1);
    second_radius = value(cf, "左右半径 / m", "second_radius", .25, .1, 1);
    height_difference = value(cf, "相邻板高差 / m", "height_difference", .05, -.1, .1);
    phase = value(cf, "初始朝向 / °", "phase", 0, -180, 180);
    car_motion = new QComboBox;
    car_motion->setObjectName("car_motion");
    car_motion->addItem("自定义", "custom");
    car_motion->addItem("左右3m平移并转动靶", "lateral_spin");
    cf->addRow("车运动状态", car_motion);
    auto * motion_hint = new QLabel("沿 Y 轴相对初始位置 ±3 m 往返，周期 2π s，同时以 7 rad/s 转动。");
    motion_hint->setWordWrap(true);
    cf->addRow(motion_hint);
    auto * motions = new QTabWidget;
    mx = new MotionEditor("x");
    my = new MotionEditor("y");
    rotation = new MotionEditor("rot", true);
    motions->addTab(mx, "X 平移");
    motions->addTab(my, "Y 平移");
    motions->addTab(rotation, "旋转");
    cf->addRow(motions);
    auto update_motion = [=] {
      const bool preset = lateral_spin();
      motions->setVisible(!preset);
      motion_hint->setVisible(preset);
    };
    connect(car_motion, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [update_motion](int) { update_motion(); });
    update_motion();
    layout->addWidget(car);
    auto * prop = new QWidget;
    auto * pf = new QFormLayout(prop);
    pf->setContentsMargins(0, 0, 0, 0);
    direction = new QComboBox;
    direction->setObjectName("direction");
    direction->addItem("正转", 1);
    direction->addItem("反转", -1);
    pf->addRow("方向", direction);
    stop_time = value(pf, "停转时刻 / s", "stop_time", 180, 0, 600);
    axis_yaw = value(pf, "靶面朝向 / °", "axis_yaw", 180, -180, 180);
    rune_a = value(pf, "变速系数 a", "rune_a", .9, .780, 1.045, 3);
    rune_omega = value(pf, "变速系数 ω", "rune_omega", 1.95, 1.884, 2.0, 3);
    active = new QCheckBox("开始激活");
    active->setObjectName("active");
    active->setChecked(true);
    pf->addRow(active);
    layout->addWidget(prop);
    shoot = new QCheckBox("固定方向发弹");
    shoot->setObjectName("shoot");
    layout->addWidget(shoot);
    auto * shot = new QWidget;
    auto * sf = new QFormLayout(shot);
    sf->setContentsMargins(0, 0, 0, 0);
    ox = value(sf, "发射点 X / m", "ox", 0, -50, 50);
    oy = value(sf, "发射点 Y / m", "oy", 0, -50, 50);
    oz = value(sf, "发射点 Z / m", "oz", .3, .01, 10);
    yaw = value(sf, "偏航 / °", "shot_yaw", 0, -180, 180);
    pitch = value(sf, "仰角 / °", "shot_pitch", 2, -80, 80);
    muzzle = value(sf, "初速 / m·s⁻¹", "muzzle", 23, 1, 50);
    shot_start = value(sf, "开始 / s", "shot_start", .2, 0, 120);
    shot_interval = value(sf, "间隔 / s", "shot_interval", .5, .05, 30);
    shot_count = new QSpinBox;
    shot_count->setObjectName("shot_count");
    shot_count->setRange(1, 200);
    shot_count->setValue(10);
    sf->addRow("弹数", shot_count);
    layout->addWidget(shot);
    connect(shoot, &QCheckBox::toggled, shot, &QWidget::setVisible);
    shot->hide();
    auto update = [=](bool change_height) {
      int k = kind->currentIndex();
      car->setVisible(k == 0);
      prop->setVisible(k != 0);
      pz->setMinimum(k == 1 ? 0 : .1);
      visible(pf, stop_time, k == 1);
      visible(pf, axis_yaw, k >= 2);
      visible(pf, rune_a, k == 3);
      visible(pf, rune_omega, k == 3);
      active->setVisible(k >= 2);
      qobject_cast<QLabel *>(form->labelForField(pz))
          ->setText(k == 1 ? "底座高度 / m" : (k >= 2 ? "机关中心高度 / m" : "装甲中心高度 / m"));
      if (change_height) pz->setValue(k >= 2 ? 2.6 : .3);
    };
    connect(kind, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [update](int) { update(true); });
    update(false);
    layout->addStretch();
  }
  bool lateral_spin() const { return car_motion->currentData().toString() == "lateral_spin"; }
  static QJsonArray lateral_spin_translation() {
    // Integrating a = 3 sin(t) with v(0) = -3 gives y(t) = -3 sin(t), without drift.
    return {QJsonObject{}, QJsonObject{{"kind", "sinusoidal"}, {"velocity", -3},
            {"acceleration", 0}, {"amplitude", 3}, {"frequency", 1}}, QJsonObject{}};
  }
  QJsonObject scenario() const {
    constexpr double degrees = 3.14159265358979323846 / 180;
    int k = kind->currentIndex();
    QJsonObject target{{"id", "target"},
                       {"kind", kind->currentData().toString()},
                       {"position", QJsonArray{px->value(), py->value(), pz->value()}}};
    if (k == 0) {
      target["armor_marking"]=marking->currentData().toString();
      target["translation"] =
          lateral_spin() ? lateral_spin_translation() :
          QJsonArray{mx->json(seed->value()), my->json(seed->value() + 1), QJsonObject{}};
      auto rot = lateral_spin() ? QJsonObject{{"kind", "velocity"}, {"velocity", 7}} :
                                 rotation->json(seed->value() + 2);
      rot["initial"] = phase->value() * degrees;
      target["rotation"] = rot;
      target["radius"] = radius->value();
      target["second_radius"] = second_radius->value();
      target["height_difference"] = height_difference->value();
      target["armor_width"] = armor->currentIndex() == 0 ? .135 : .230;
      target["armor_height"] = armor->currentIndex() == 0 ? .125 : .127;
    } else {
      target["direction"] = direction->currentData().toInt();
      if (k == 1)
        target["stop_time"] = stop_time->value();
      else {
        target["axis_yaw"] = axis_yaw->value() * degrees;
        target["seed"] = seed->value();
        target["initially_active"] = active->isChecked();
        if (k == 3) {
          target["rune_a"] = rune_a->value();
          target["rune_omega"] = rune_omega->value();
        }
      }
    }
    QJsonArray shots;
    if (shoot->isChecked()) {
      double y = yaw->value() * degrees, p = pitch->value() * degrees, v = muzzle->value();
      for (int i = 0; i < shot_count->value(); ++i) {
        double time = std::round((shot_start->value() + i * shot_interval->value()) * 1000) / 1000;
        if (time >= duration->value()) break;
        shots.append(
            QJsonObject{{"id", QString("shot_%1").arg(i)},
                        {"time", time},
                        {"origin", QJsonArray{ox->value(), oy->value(), oz->value()}},
                        {"velocity", QJsonArray{v * std::cos(p) * std::cos(y),
                                                v * std::cos(p) * std::sin(y), v * std::sin(p)}}});
      }
    }
    return {{"schema_version", 1},           {"name", kind->currentText()},
            {"duration", duration->value()}, {"sample_period", .02},
            {"targets", QJsonArray{target}}, {"shots", shots}};
  }
  void settings(bool save) {
    QSettings s;
    s.beginGroup("target_editor");
    // Restore target kind first because its UI change sets the default height.
    if (!save && s.contains("target_kind")) kind->setCurrentIndex(s.value("target_kind").toInt());
    for (auto * w : findChildren<QWidget *>()) {
      auto id = w->objectName();
      if (id.isEmpty() || id.startsWith("qt_")) continue;
      if (auto * v = qobject_cast<QDoubleSpinBox *>(w)) {
        if (save)
          s.setValue(id, v->value());
        else if (s.contains(id))
          v->setValue(s.value(id).toDouble());
      } else if (auto * v = qobject_cast<QSpinBox *>(w)) {
        if (save)
          s.setValue(id, v->value());
        else if (s.contains(id))
          v->setValue(s.value(id).toInt());
      } else if (auto * v = qobject_cast<QComboBox *>(w)) {
        if (save)
          s.setValue(id, v->currentIndex());
        else if (s.contains(id))
          v->setCurrentIndex(s.value(id).toInt());
      } else if (auto * v = qobject_cast<QCheckBox *>(w)) {
        if (save)
          s.setValue(id, v->isChecked());
        else if (s.contains(id))
          v->setChecked(s.value(id).toBool());
      }
    }
  }
};
}  // namespace rm_sim::ui
