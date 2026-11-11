#pragma once
#include "rm_sim/sensors.hpp"
#include "rm_sim/armor_markings.hpp"

namespace rm_sim::camera_detail {
using Color=std::array<uint8_t,3>; // BGR
// Shared receiving surfaces, shell occlusion, light layout and markings for both renderers.
template<class Emit> void planes(const Snapshot& snapshot,const Pose& pose,const ImageConfig& config,Emit emit) {
  const Color light=config.color=="blue" ? Color{255,30,10}:Color{10,30,255};
  for(const auto& target:snapshot.targets)for(const auto& s:target.surfaces) {
    emit(s.center-s.normal*.004,s.right,s.up,s.width/2+.008,s.height/2,Color{14,16,19},s.disk_radius>0?2:0,nullptr);
    if(dot(s.normal,pose.translation-s.center)<=0)continue;
    if(s.disk_radius>0) {
      emit(s.center,s.right,s.up,s.disk_radius,s.disk_radius,s.active?light:Color{65,65,65},2,nullptr);
      emit(s.center+s.normal*.001,s.right,s.up,s.disk_radius*.87,s.disk_radius*.87,Color{30,30,30},2,nullptr);
    }else {
      const auto& marking=armor_marking(target);
      emit(s.center+s.normal*.001,s.right,s.up,ArmorMarking::physical_width/2,
        ArmorMarking::physical_height/2,Color{240,240,240},1,&marking);
      const double spacing=s.width>.18?config.big_light_spacing:config.small_light_spacing;
      for(double side:{-1.,1.})emit(s.center+s.right*(side*spacing/2)+s.normal*.002,
        s.right,s.up,config.light_width/2,config.light_length/2,light,4,nullptr);
    }
  }
}
std::shared_ptr<Image> render_gpu(const Snapshot&,const Pose&,const CameraCalibration&,const ImageConfig&);
}
