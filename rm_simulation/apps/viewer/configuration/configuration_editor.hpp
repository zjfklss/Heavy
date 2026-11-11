#pragma once
#include "json_fields.hpp"
#include "rm_sim/runner.hpp"
#include "rm_sim/adapters/profile.hpp"
#include <QScrollArea>
#include <QStackedWidget>
#include <QPushButton>
#include <QTabBar>

namespace rm_sim::ui {
class ConfigurationEditor : public QWidget {
 public:
  ScenarioEditor* target;
  QTabBar* section;
  QTabWidget *camera_tabs, *actuator_tabs, *scheduler_tabs;
  JsonFields fields;
  ConfigurationEditor() {
    auto* layout=new QVBoxLayout(this); layout->setContentsMargins(0,0,0,0);
    section=new QTabBar; section->setObjectName("configuration_section");
    section->setExpanding(true); section->setDrawBase(false);
    for (const auto& name:QStringList{"目标","相机","云台","调度"}) section->addTab(name);
    layout->addWidget(section);
    auto* stack=new QStackedWidget; layout->addWidget(stack,1);
    connect(section,&QTabBar::currentChanged,stack,[stack](int index){ if(index<stack->count()) stack->setCurrentIndex(index); });
    auto* target_page=new QWidget; auto* target_layout=new QVBoxLayout(target_page);
    target_layout->setContentsMargins(0,0,0,0);
    target_info_=new QLabel; target_info_->setWordWrap(true); target_info_->hide(); target_layout->addWidget(target_info_);
    preserve_shots_=new QCheckBox; preserve_shots_->hide(); target_layout->addWidget(preserve_shots_);
    target=new ScenarioEditor; target_layout->addWidget(target);
    connect(preserve_shots_,&QCheckBox::toggled,this,[this](bool on){
      target->shoot->setEnabled(!on);
      if (on) target->shoot->setChecked(false);
    });
    stack->addWidget(scroll(target_page));
    camera_tabs=new QTabWidget; stack->addWidget(camera_tabs);
    actuator_tabs=new QTabWidget; stack->addWidget(actuator_tabs);
    scheduler_tabs=new QTabWidget; stack->addWidget(scheduler_tabs);
    const SensorConfig sensor; const ActuatorConfig actuator; const WorldConfig world; const PixelServoConfig algorithm;
    auto* sample=page(camera_tabs,"采样");
    fields.boolean(sample,"启用相机","sensor.enabled",sensor.enabled);
    auto* output=fields.choice(sample,"输出","sensor.output",{"相机图像（BGR8）","几何特征（兼容）"},{"image","features"});
    fields.text(sample,"传感器名称","sensor.id",QString::fromStdString(sensor.id));
    fields.number(sample,"采样率 / Hz","sensor.frequency",sensor.frequency,.001,1e6);
    fields.number(sample,"观测延迟 / ms","sensor.latency",sensor.latency,0,600,3,1000);
    auto* feature_noise=fields.number(sample,"特征噪声 σ / px","sensor.pixel_sigma",sensor.pixel_sigma,0,1e6);
    auto* dropout=fields.number(sample,"漏检概率","sensor.dropout",sensor.dropout,0,1);
    auto* false_positive=fields.number(sample,"误检概率 / 帧","sensor.false_positive_probability",sensor.false_positive_probability,0,1);
    auto* occlusion=fields.boolean(sample,"受击面遮挡检测","sensor.surface_occlusion",sensor.surface_occlusion);
    seed(sample,"随机种子","sensor.seed",sensor.seed);
    auto* calibration=page(camera_tabs,"内参");
    fields.text(calibration,"标定版本","sensor.calibration.id",QString::fromStdString(sensor.calibration.id));
    fields.number(calibration,"图像宽度 / px","sensor.calibration.width",sensor.calibration.width,1,65536,0);
    fields.number(calibration,"图像高度 / px","sensor.calibration.height",sensor.calibration.height,1,65536,0);
    fields.number(calibration,"fx / px","sensor.calibration.fx",sensor.calibration.fx,.000001,1e9);
    fields.number(calibration,"fy / px","sensor.calibration.fy",sensor.calibration.fy,.000001,1e9);
    fields.number(calibration,"cx / px","sensor.calibration.cx",sensor.calibration.cx,-1e9,1e9);
    fields.number(calibration,"cy / px","sensor.calibration.cy",sensor.calibration.cy,-1e9,1e9);
    auto* imaging=page(camera_tabs,"成像");
    note(imaging,"按内参页的原生分辨率生成 BGR 像素，包含灯条、标识和板面遮挡。自动模式优先使用硬件 GPU；实时图像直接传送，不经 PNG。");
    std::vector<QWidget*> image_fields;
    image_fields.push_back(fields.choice(imaging,"渲染后端","sensor.image.renderer",{"自动（优先 GPU）","GPU（必须可用）","CPU"},{"auto","gpu","cpu"}));
    image_fields.push_back(fields.choice(imaging,"灯条颜色","sensor.image.color",{"蓝色","红色"},{"blue","red"}));
    image_fields.push_back(fields.number(imaging,"图像噪声 σ / 灰度","sensor.image.noise_sigma",0,0,255));
    image_fields.push_back(fields.number(imaging,"灯条长度 / mm","sensor.image.light_length",sensor.image.light_length,.0001,.3,3,1000));
    image_fields.push_back(fields.number(imaging,"灯条宽度 / mm","sensor.image.light_width",sensor.image.light_width,.0001,.3,3,1000));
    image_fields.push_back(fields.number(imaging,"小装甲灯距 / mm","sensor.image.small_light_spacing",sensor.image.small_light_spacing,.0001,1,3,1000));
    image_fields.push_back(fields.number(imaging,"大装甲灯距 / mm","sensor.image.big_light_spacing",sensor.image.big_light_spacing,.0001,1,3,1000));
    auto* appearance=page(camera_tabs,"特征");
    note(appearance,"留空使用几何包络。自定义特征按目标 ID 配置，点坐标单位为 m。");
    auto* templates=fields.json(appearance,"特征模板","sensor.appearances",QJsonObject{});
    auto output_changed=[=](int index){
      for(QWidget* w:std::vector<QWidget*>{feature_noise,dropout,false_positive,occlusion,templates}) w->setEnabled(index==1);
      for(auto* w:image_fields) w->setEnabled(index==0);
    };
    connect(output,qOverload<int>(&QComboBox::currentIndexChanged),this,output_changed);output_changed(output->currentIndex());
    auto* motion=page(actuator_tabs,"运动");
    auto* ideal=fields.boolean(motion,"理想响应（立即到达）","actuator.ideal",actuator.ideal);
    auto* wrapping=fields.boolean(motion,"Yaw 连续回绕","actuator.wrap_yaw",actuator.wrap_yaw);
    auto* align=fields.boolean(motion,"启动时朝向装甲板","actuator.align_on_start",true);
    auto* initial_yaw=angle(motion,"初始 yaw / °","initial_yaw",actuator.initial_yaw);
    auto* initial_pitch=angle(motion,"初始 pitch / °","initial_pitch",actuator.initial_pitch);
    auto enable_initial=[=](bool on){initial_yaw->setEnabled(!on);initial_pitch->setEnabled(!on);};
    connect(align,&QCheckBox::toggled,this,enable_initial);enable_initial(align->isChecked());
    auto* ymin=angle(motion,"Yaw 下限 / °","yaw_min",actuator.yaw_min);
    auto* ymax=angle(motion,"Yaw 上限 / °","yaw_max",actuator.yaw_max);
    angle(motion,"Pitch 下限 / °","pitch_min",actuator.pitch_min);
    angle(motion,"Pitch 上限 / °","pitch_max",actuator.pitch_max);
    auto* speed=fields.number(motion,"角速度上限 / rad·s⁻¹","actuator.max_speed",actuator.max_speed,.000001,1e6);
    auto* acceleration=fields.number(motion,"角加速度 / rad·s⁻²","actuator.max_acceleration",actuator.max_acceleration,.000001,1e6);
    auto* response=fields.number(motion,"响应时间 / ms","actuator.response_time",actuator.response_time,0,600,3,1000);
    fields.number(motion,"命令延迟 / ms","actuator.command_latency",actuator.command_latency,0,600,3,1000);
    auto enable_dynamics=[=](bool on){speed->setEnabled(!on);acceleration->setEnabled(!on);response->setEnabled(!on);};
    connect(ideal,&QCheckBox::toggled,this,enable_dynamics); enable_dynamics(ideal->isChecked());
    auto enable_limits=[=](bool on){ymin->setEnabled(!on);ymax->setEnabled(!on);};
    connect(wrapping,&QCheckBox::toggled,this,enable_limits); enable_limits(wrapping->isChecked());
    auto* mounts=page(actuator_tabs,"安装");
    fields.pose(mounts,"基座","actuator.base_pose",actuator.base_pose);
    fields.vector(mounts,"Yaw 轴偏置","actuator.yaw_offset",{0,0,0});
    fields.vector(mounts,"Pitch 轴偏置","actuator.pitch_offset",{0,0,0});
    fields.pose(mounts,"相机","actuator.camera_mount",actuator.camera_mount);
    fields.pose(mounts,"枪口","actuator.muzzle_mount",actuator.muzzle_mount);
    auto* fire=page(actuator_tabs,"发射");
    fields.number(fire,"弹速 / m·s⁻¹","actuator.muzzle_speed",actuator.muzzle_speed,.000001,1e6);
    fields.number(fire,"弹速噪声 σ / m·s⁻¹","actuator.muzzle_speed_sigma",actuator.muzzle_speed_sigma,0,1e6);
    fields.number(fire,"准备延迟 / ms","actuator.fire_latency",actuator.fire_latency,0,600,3,1000);
    fields.number(fire,"最小发弹间隔 / ms","actuator.fire_interval",actuator.fire_interval,.000001,600,3,1000);
    fields.number(fire,"弹丸寿命 / s","actuator.shot_lifetime",actuator.shot_lifetime,.000001,600);
    fields.boolean(fire,"继承枪口运动速度","actuator.inherit_muzzle_velocity",actuator.inherit_muzzle_velocity);
    seed(fire,"弹速随机种子","actuator.seed",actuator.seed);
    auto* feedback=page(actuator_tabs,"反馈");
    fields.boolean(feedback,"启用编码器反馈","actuator.feedback_enabled",actuator.feedback_enabled);
    fields.number(feedback,"采样率 / Hz","actuator.feedback_frequency",actuator.feedback_frequency,.001,1e6);
    fields.number(feedback,"反馈延迟 / ms","actuator.feedback_latency",actuator.feedback_latency,0,600,3,1000);
    fields.number(feedback,"角度噪声 σ / rad","actuator.encoder_sigma",actuator.encoder_sigma,0,100);
    seed(feedback,"反馈随机种子","actuator.feedback_seed",actuator.feedback_seed);
    auto* clock=page(scheduler_tabs,"时间");
    fields.number(clock,"物理步长 / ms","scene.world.timestep",world.timestep,.000001,.001,6,1000);
    fields.number(clock,"算法更新率 / Hz","runner.algorithm_frequency",200,.001,1e6);
    fields.number(clock,"日志采样率 / Hz","runner.log_frequency",50,.001,1e6);
    fields.number(clock,"场景采样周期 / ms","scene.sample_period",.02,.000001,600,6,1000);
    auto* algo=page(scheduler_tabs,"算法");
    auto* kind=fields.choice(algo,"控制器","algorithm.kind",{"不使用算法","像素伺服（特征模式）","外部自瞄（协议连接）"},{"none","pixel_servo","external"});
    note(algo,"外部模式只交换图像、反馈和指令。请先单独启动自瞄端监听，再在此选择协议并运行；仿真不加载算法代码。");
    std::vector<QWidget*> algorithm_fields;
    algorithm_fields.push_back(fields.number(algo,"比例增益","algorithm.gain",algorithm.gain,.000001,1));
    algorithm_fields.push_back(fields.boolean(algo,"允许算法开火","algorithm.fire",algorithm.fire));
    algorithm_fields.push_back(fields.number(algo,"开火角误差 / rad","algorithm.fire_tolerance",algorithm.fire_tolerance,0,pi));
    algorithm_fields.push_back(fields.number(algo,"开火请求间隔 / ms","algorithm.fire_interval",algorithm.fire_interval,.000001,600,3,1000));
    algorithm_fields.push_back(fields.number(algo,"观测最大时龄 / ms","algorithm.max_observation_age",algorithm.max_observation_age,.000001,600,3,1000));
    QStringList profile_names,profile_ids;
    for(const auto& entry:adapters::profiles()) {
      profile_names<<QString::fromStdString(entry.label);profile_ids<<QString::fromStdString(entry.id);
    }
    std::vector<QWidget*> external_fields;
    external_fields.push_back(fields.choice(algo,"算法协议","algorithm.adapter",profile_names,profile_ids));
    external_fields.push_back(fields.text(algo,"自瞄端 IPv4","algorithm.host","127.0.0.1"));
    external_fields.push_back(fields.number(algo,"TCP 端口","algorithm.port",17001,1,65535,0));
    external_fields.push_back(fields.number(algo,"连接超时 / ms","algorithm.connect_timeout",5,.001,60,3,1000));
    external_fields.push_back(fields.number(algo,"单步超时 / ms","algorithm.step_timeout",1,.001,60,3,1000));
    auto controller_changed=[=](int index){
      for(auto* w:algorithm_fields){w->setEnabled(index==1);visible(algo,w,index==1);}
      for(auto* w:external_fields){w->setEnabled(index==2);visible(algo,w,index==2);}
      if(index==2)output->setCurrentIndex(0);
    };
    connect(kind,qOverload<int>(&QComboBox::currentIndexChanged),this,controller_changed);
    controller_changed(kind->currentIndex());
    connect(output,qOverload<int>(&QComboBox::currentIndexChanged),kind,[kind](int index){
      if(index==0 && kind->currentData().toString()=="pixel_servo")kind->setCurrentIndex(0);
    });
    auto* physics=page(scheduler_tabs,"物理");
    fields.number(physics,"重力 X / m·s⁻²","scene.world.gravity.0",world.gravity.x,-1e6,1e6);
    fields.number(physics,"重力 Y / m·s⁻²","scene.world.gravity.1",world.gravity.y,-1e6,1e6);
    fields.number(physics,"重力 Z / m·s⁻²","scene.world.gravity.2",world.gravity.z,-1e6,1e6);
    fields.number(physics,"地面高度 / m","scene.world.ground_z",world.ground_z,-1e6,1e6);
    fields.number(physics,"有效法向速度 / m·s⁻¹","scene.world.minimum_normal_speed",world.minimum_normal_speed,0,1e6);
    fields.number(physics,"检测间隔 / ms","scene.world.detection_interval",world.detection_interval,0,600,3,1000);
    auto* events=page(scheduler_tabs,"事件");
    fields.json(events,"预置云台命令","commands",QJsonArray{});
    fields.json(events,"场景事件","scene.events",QJsonArray{});
    load({{"closed_loop_version",1},{"scene",target->scenario()}});
  }
  QJsonObject configuration() const {
    auto result=fields.current();
    auto scene=current_scene();
    // Scene fields with dedicated controls take their values from those controls.
    auto controlled=result["scene"].toObject();
    for (const auto& key:{"world","sample_period","events"}) if (controlled.contains(key)) scene[key]=controlled[key];
    result["scene"]=scene; result["closed_loop_version"]=1;
    auto algorithm=result["algorithm"].toObject();
    auto kind=algorithm["kind"].toString();QJsonObject selected{{"kind",kind}};
    QStringList keys=kind=="external" ? QStringList{"adapter","host","port","connect_timeout","step_timeout"} :
      kind=="pixel_servo" ? QStringList{"gain","fire","fire_tolerance","fire_interval","max_observation_age"}:QStringList{};
    for(const auto& key:keys)if(algorithm.contains(key))selected[key]=algorithm[key];
    result["algorithm"]=selected;
    return result;
  }
  void load(QJsonObject root) {
    if (!root.contains("closed_loop_version")) root={{"closed_loop_version",1},{"scene",root},{"algorithm",QJsonObject{{"kind","none"}}}};
    if (!root["sensor"].toObject().contains("output") && root["algorithm"].toObject()["kind"].toString()=="pixel_servo")
      root=set_path(root,{"sensor","output"},"features").toObject();
    if (!root["algorithm"].toObject().contains("kind"))
      root=set_path(root,{"algorithm","kind"},root["sensor"].toObject()["output"].toString()=="features" ? "pixel_servo":"none").toObject();
    auto scene=root["scene"].toObject();
    if (scene["targets"].toArray().isEmpty()) throw std::runtime_error("场景至少需要一个目标");
    if (!root["runner"].toObject().contains("log_frequency")) {
      double period=scene.value("sample_period").toDouble(.02);
      if (!(period>0)) throw std::runtime_error("采样周期必须大于 0");
      root=set_path(root,{"runner","log_frequency"},1/period).toObject();
    }
    load_target(scene);
    fields.load(root);
  }
  QJsonObject current_scene() const {
    auto current=target->scenario();
    if (scene_source_.isEmpty()) return current;
    auto result=scene_source_;
    auto original_targets=result["targets"].toArray();
    auto generated=current["targets"].toArray()[0].toObject();
    auto before=target_baseline_["targets"].toArray()[0].toObject();
    auto original=original_targets[0].toObject();
    auto merged=generated["kind"]==before["kind"] ? edited_json(original,before,generated).toObject() : generated;
    if (generated["kind"].toString()=="car" && target->lateral_spin()) {
      // A preset replaces imported axis offsets/phases too, so they cannot introduce drift.
      merged["translation"]=generated["translation"];
      auto rotation=generated["rotation"].toObject();
      if (generated["kind"]==before["kind"] &&
          rotation["initial"]==before["rotation"].toObject()["initial"] &&
          original["rotation"].toObject().contains("initial"))
        rotation["initial"]=original["rotation"].toObject()["initial"];
      merged["rotation"]=rotation;
    }
    merged["id"]=original["id"]; original_targets[0]=merged; result["targets"]=original_targets;
    if (current["duration"]!=target_baseline_["duration"]) result["duration"]=current["duration"];
    if (current["name"]!=target_baseline_["name"] && result["name"]==target_baseline_["name"]) result["name"]=current["name"];
    if (!preserve_shots_->isChecked()) result["shots"]=current["shots"];
    return result;
  }
 private:
  QLabel* target_info_;
  QCheckBox* preserve_shots_;
  QJsonObject scene_source_,target_baseline_;
  static QScrollArea* scroll(QWidget* widget) {
    auto* area=new QScrollArea; area->setWidgetResizable(true); area->setFrameShape(QFrame::NoFrame);
    area->setWidget(widget); return area;
  }
  static QFormLayout* page(QTabWidget* tabs,QString name) {
    auto* widget=new QWidget; auto* layout=new QFormLayout(widget);
    layout->setContentsMargins(8,12,8,12); layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    layout->setRowWrapPolicy(QFormLayout::WrapLongRows);
    tabs->addTab(scroll(widget),name); return layout;
  }
  static void note(QFormLayout* form,QString text) {
    auto* label=new QLabel(text); label->setWordWrap(true); label->setStyleSheet("color:#64748b;font-size:11px;"); form->addRow(label);
  }
  QDoubleSpinBox* angle(QFormLayout* form,QString label,QString key,double initial) {
    return fields.number(form,label,"actuator."+key,initial,-1e6,1e6,6,180/pi);
  }
  void seed(QFormLayout* form,QString label,QString path,uint64_t initial) {
    fields.number(form,label,path,double(initial),0,9007199254740991.,0);
  }
  static void set(QDoubleSpinBox* box,double value) {
    box->setRange(std::min(box->minimum(),value),std::max(box->maximum(),value)); box->setValue(value);
  }
  static void motion(MotionEditor* editor,QJsonObject m) {
    int index=editor->kind->findData(m.value("kind").toString("stationary"));
    if(index<0) throw std::runtime_error("目标运动类型不受支持");
    editor->kind->setCurrentIndex(index);
    set(editor->velocity,m.value("velocity").toDouble(0));
    set(editor->acceleration,m.value("acceleration").toDouble(0));
    set(editor->jerk,m.value("jerk").toDouble(0));
    set(editor->amplitude,m.value("amplitude").toDouble(1));
    set(editor->frequency,m.value("frequency").toDouble(1));
  }
  void load_target(const QJsonObject& scene) {
    auto targets=scene["targets"].toArray(); auto t=targets[0].toObject();
    int k=target->kind->findData(t["kind"].toString());
    if(k<0) throw std::runtime_error("目标类型不受支持");
    target->kind->setCurrentIndex(k);
    auto position=t["position"].toArray();
    if(position.size()!=3) throw std::runtime_error("目标位置需要三个分量");
    set(target->px,position[0].toDouble()); set(target->py,position[1].toDouble()); set(target->pz,position[2].toDouble());
    set(target->duration,scene["duration"].toDouble());
    set(target->radius,t.value("radius").toDouble(.25)); set(target->second_radius,t.value("second_radius").toDouble(.25));
    set(target->height_difference,t.value("height_difference").toDouble(.05));
    target->armor->setCurrentIndex(t.value("armor_width").toDouble(.135)>.18 ? 1:0);
    int marking=target->marking->findData(t.value("armor_marking").toString("3"));
    if(k==0 && marking<0)throw std::runtime_error("不支持的 2026 装甲标识");
    target->marking->setCurrentIndex(marking<0 ? 2:marking);
    auto translation=t["translation"].toArray();
    motion(target->mx,translation.size()==3 ? translation[0].toObject():QJsonObject{});
    motion(target->my,translation.size()==3 ? translation[1].toObject():QJsonObject{});
    motion(target->rotation,t["rotation"].toObject());
    auto rotation=t["rotation"].toObject(); rotation.remove("initial");
    target->car_motion->setCurrentIndex(k==0 && translation==ScenarioEditor::lateral_spin_translation() &&
      rotation==QJsonObject{{"kind","velocity"},{"velocity",7}} ? 1:0);
    set(target->phase,t["rotation"].toObject().value("initial").toDouble()*180/pi);
    target->direction->setCurrentIndex(t.value("direction").toInt(1)==1 ? 0:1);
    set(target->stop_time,t.value("stop_time").toDouble(180));
    set(target->axis_yaw,t.value("axis_yaw").toDouble(pi)*180/pi);
    set(target->rune_a,t.value("rune_a").toDouble(.9)); set(target->rune_omega,t.value("rune_omega").toDouble(1.95));
    target->active->setChecked(t.value("initially_active").toBool(true));
    int seed=t.value("seed").toInt(27);
    if(k==0 && translation.size()==3) seed=translation[0].toObject().value("seed").toInt(seed);
    target->seed->setValue(seed);
    int count=scene["shots"].toArray().size(); target->shoot->setChecked(false);
    preserve_shots_->setText(QString("保留文件中的 %1 发预置弹丸").arg(count));
    preserve_shots_->setVisible(count>0); preserve_shots_->setChecked(count>0);
    target_info_->setText(QString("编辑目标：%1%2").arg(t["id"].toString(),targets.size()>1 ? QString("（另 %1 个目标保留）").arg(targets.size()-1):QString()));
    target_info_->show(); scene_source_=scene; target_baseline_=target->scenario();
  }
};
} // namespace rm_sim::ui
