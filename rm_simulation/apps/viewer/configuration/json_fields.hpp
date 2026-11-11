#pragma once
#include "scenario_editor.hpp"
#include "rm_sim/protocol.hpp"
#include <QGridLayout>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <functional>
#include <vector>
#include <stdexcept>

namespace rm_sim::ui {
inline QJsonValue at_path(QJsonValue root, const QStringList& path) {
  for (const auto& key:path) {
    if (root.isArray()) { auto a=root.toArray(); int i=key.toInt(); root=(i>=0 && i<a.size()) ? a[i] : QJsonValue(QJsonValue::Undefined); }
    else root=root.toObject().value(key);
  }
  return root;
}
inline QJsonValue set_path(QJsonValue root, const QStringList& path, QJsonValue v, int i=0) {
  if (i==path.size()) return v;
  bool index=false; int n=path[i].toInt(&index);
  if (index) {
    auto a=root.toArray(); while (a.size()<=n) a.append(QJsonValue());
    a[n]=set_path(a[n],path,v,i+1); return a;
  }
  auto o=root.toObject(); o[path[i]]=set_path(o[path[i]],path,v,i+1); return o;
}
// Overlay only actual form edits. Preserve fields and precision not represented by the form.
inline QJsonValue edited_json(QJsonValue original,QJsonValue before,QJsonValue after) {
  if (before==after) return original;
  if (before.isObject() && after.isObject()) {
    auto o=original.toObject(), b=before.toObject(), a=after.toObject();
    for (auto it=b.begin();it!=b.end();++it) if (!a.contains(it.key())) o.remove(it.key());
    for (auto it=a.begin();it!=a.end();++it) {
      if (!b.contains(it.key())) o[it.key()]=it.value();
      else if (b[it.key()]!=it.value()) o[it.key()]=edited_json(o[it.key()],b[it.key()],it.value());
    }
    return o;
  }
  if (before.isArray() && after.isArray() && before.toArray().size()==after.toArray().size() &&
      original.toArray().size()==after.toArray().size()) {
    auto o=original.toArray(), b=before.toArray(), a=after.toArray();
    for (int i=0;i<a.size();++i) o[i]=edited_json(o[i],b[i],a[i]);
    return o;
  }
  return after;
}
class JsonFields {
 public:
  struct Field {
    QStringList path;
    QJsonValue fallback, display;
    std::function<QJsonValue()> read;
    std::function<void(QJsonValue)> write;
  };
  std::vector<Field> fields;
  QJsonObject source;
  QDoubleSpinBox* number(QFormLayout* form,QString label,QString path,double initial,
      double minimum,double maximum,int decimals=6,double scale=1) {
    auto* box=value(form,label,path,initial*scale,minimum*scale,maximum*scale,decimals);
    box->setMinimumWidth(118);
    if (decimals==0 || scale==1000) box->setSingleStep(1);
    fields.push_back({path.split('.'),initial,{},[=]{return QJsonValue(box->value()/scale);},[=](QJsonValue v){
      if (!v.isDouble() || !std::isfinite(v.toDouble()) || v.toDouble()<minimum || v.toDouble()>maximum ||
          (decimals==0 && std::floor(v.toDouble())!=v.toDouble()))
        throw std::runtime_error((path+" 超出界面支持的范围").toStdString());
      box->setValue(v.toDouble()*scale);
    }});
    return box;
  }
  QCheckBox* boolean(QFormLayout* form,QString label,QString path,bool initial) {
    auto* box=new QCheckBox(label); box->setObjectName(path); box->setChecked(initial); form->addRow(box);
    fields.push_back({path.split('.'),initial,{},[=]{return QJsonValue(box->isChecked());},[=](QJsonValue v){
      if (!v.isBool()) throw std::runtime_error((path+" 必须为布尔值").toStdString());
      box->setChecked(v.toBool());
    }}); return box;
  }
  QLineEdit* text(QFormLayout* form,QString label,QString path,QString initial) {
    auto* edit=new QLineEdit(initial); edit->setObjectName(path); form->addRow(label,edit);
    fields.push_back({path.split('.'),initial,{},[=]{return QJsonValue(edit->text());},[=](QJsonValue v){
      if (!v.isString()) throw std::runtime_error((path+" 必须为文本").toStdString());
      edit->setText(v.toString());
    }}); return edit;
  }
  QComboBox* choice(QFormLayout* form,QString label,QString path,QStringList names,QStringList codes) {
    auto* box=new QComboBox; box->setObjectName(path);
    for (int i=0;i<names.size();++i) box->addItem(names[i],codes[i]);
    form->addRow(label,box);
    fields.push_back({path.split('.'),codes[0],{},[=]{return QJsonValue(box->currentData().toString());},[=](QJsonValue v){
      int index=box->findData(v.toString());
      if (index<0) throw std::runtime_error((path+" 不受支持").toStdString());
      box->setCurrentIndex(index);
    }}); return box;
  }
  QPlainTextEdit* json(QFormLayout* form,QString label,QString path,QJsonValue initial) {
    auto* edit=new QPlainTextEdit; edit->setObjectName(path); edit->setMinimumHeight(140);
    edit->setLineWrapMode(QPlainTextEdit::NoWrap); form->addRow(new QLabel(label)); form->addRow(edit);
    fields.push_back({path.split('.'),initial,{},[=]{
      QJsonParseError error; auto d=QJsonDocument::fromJson(edit->toPlainText().toUtf8(),&error);
      if (error.error!=QJsonParseError::NoError || (initial.isObject() ? !d.isObject() : !d.isArray()))
        throw std::runtime_error((label+"：JSON 格式错误").toStdString());
      return initial.isObject() ? QJsonValue(d.object()) : QJsonValue(d.array());
    },[=](QJsonValue v){
      if (initial.isObject() ? !v.isObject() : !v.isArray()) throw std::runtime_error("JSON 类型错误");
      edit->setPlainText(QString::fromUtf8((v.isObject() ? QJsonDocument(v.toObject()) : QJsonDocument(v.toArray())).toJson()));
    }}); return edit;
  }
  void vector(QFormLayout* form,QString label,QString path,std::array<double,3> initial) {
    for (int i=0;i<3;++i) number(form,label+" "+QStringList{"X","Y","Z"}[i]+" / m",
      path+"."+QString::number(i),initial[i],-1e6,1e6,6);
  }
  void pose(QFormLayout* form,QString label,QString path,const Pose& p) {
    vector(form,label,path+".translation",{p.translation.x,p.translation.y,p.translation.z});
    auto* toggle=new QCheckBox(label+"旋转矩阵"); form->addRow(toggle);
    auto* container=new QWidget; auto* grid=new QGridLayout(container); grid->setContentsMargins(0,0,0,0);
    const Vec3 columns[]={p.rotation.x,p.rotation.y,p.rotation.z};
    for (int col=0;col<3;++col) {
      grid->addWidget(new QLabel(QString("列 %1").arg(col+1)),0,col);
      const double entries[]={columns[col].x,columns[col].y,columns[col].z};
      for (int row=0;row<3;++row) {
        auto* temporary=new QFormLayout;
        auto* box=number(temporary,"",path+".rotation."+QString::number(col)+"."+QString::number(row),entries[row],-1,1,9);
        box->setMinimumWidth(0); box->setButtonSymbols(QAbstractSpinBox::NoButtons);
        box->setStyleSheet("QDoubleSpinBox { padding:4px; }");
        delete temporary->labelForField(box); temporary->removeWidget(box); delete temporary; grid->addWidget(box,row+1,col);
      }
    }
    form->addRow(container); container->hide();
    QObject::connect(toggle,&QCheckBox::toggled,container,&QWidget::setVisible);
  }
  void load(QJsonObject root) {
    source=root;
    for (auto& f:fields) {
      auto v=at_path(source,f.path);
      if (v.isUndefined() || v.isNull()) { v=f.fallback; source=set_path(source,f.path,v).toObject(); }
      f.write(v); f.display=f.read();
    }
  }
  QJsonObject current() const {
    auto result=source;
    for (const auto& f:fields) {
      auto v=f.read();
      if (v!=f.display) result=set_path(result,f.path,v).toObject();
    }
    return result;
  }
};
} // namespace rm_sim::ui
