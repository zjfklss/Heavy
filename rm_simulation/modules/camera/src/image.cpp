#include "rm_sim/image.hpp"
#include "rm_sim/sensors.hpp"
#include "rm_sim/armor_markings.hpp"
#include "camera_planes.hpp"
#include <iostream>
#include <limits>
#include <random>

namespace rm_sim {
void ImageConfig::validate() const {
  ensure(color=="blue" || color=="red", "Image light color must be red or blue");
  ensure(renderer=="auto" || renderer=="gpu" || renderer=="cpu","Camera renderer must be auto, gpu or cpu");
  ensure(std::isfinite(light_length) && light_length>0 && light_length<=.3 &&
    std::isfinite(light_width) && light_width>0 && light_width<light_length &&
    std::isfinite(small_light_spacing) && small_light_spacing>0 && small_light_spacing<=1 &&
    std::isfinite(big_light_spacing) && big_light_spacing>0 && big_light_spacing<=1 &&
    std::isfinite(noise_sigma) && noise_sigma>=0 && noise_sigma<=255,"Invalid image appearance");
}
namespace {
using Color=std::array<uint8_t,3>;
class Raster {
 public:
  int w,h;
  double fx,fy,cx,cy;
  Pose pose;
  std::vector<uint8_t> pixels;
  std::vector<float> depth;
  Raster(const CameraCalibration& k,const Pose& p):w(k.width),h(k.height),
    fx(k.fx),fy(k.fy),cx(k.cx),cy(k.cy),pose(p),pixels(size_t(w)*h*3,28),depth(size_t(w)*h,0) {}
  // Perspective-correct ray/plane rasterization, including near clipping and per-pixel depth.
  void plane(Vec3 c,Vec3 right,Vec3 up,double hw,double hh,Color color,int texture=0,
      const ArmorMarking* marking=nullptr) {
    auto n=cross(right,up);
    const auto cc=pose.inverse(c),r=pose.rotation.inverse(right),u=pose.rotation.inverse(up),normal=pose.rotation.inverse(n);
    double left=w,top=h,endx=0,endy=0;
    std::vector<Vec3> polygon{cc-r*hw-u*hh,cc+r*hw-u*hh,cc+r*hw+u*hh,cc-r*hw+u*hh}, clipped;
    for(size_t i=0;i<4;++i) {
      auto a=polygon[i],b=polygon[(i+1)%4];
      if(a.z>=.01) clipped.push_back(a);
      if((a.z>=.01)!=(b.z>=.01)) clipped.push_back(a+(b-a)*((.01-a.z)/(b.z-a.z)));
    }
    if(clipped.empty()) return;
    for(auto p:clipped) { double x=fx*p.x/p.z+cx,y=fy*p.y/p.z+cy;
      left=std::min(left,x);top=std::min(top,y);endx=std::max(endx,x);endy=std::max(endy,y); }
    int x0=int(std::clamp(std::floor(left),0.,double(w))),y0=int(std::clamp(std::floor(top),0.,double(h)));
    int x1=int(std::clamp(std::ceil(endx),0.,double(w))),y1=int(std::clamp(std::ceil(endy),0.,double(h)));
    double offset=dot(cc,normal);
    for(int y=y0;y<y1;++y) for(int x=x0;x<x1;++x) {
      Vec3 ray{(x+.5-cx)/fx,(y+.5-cy)/fy,1};
      double denom=dot(ray,normal); if(std::abs(denom)<1e-12) continue;
      double z=offset/denom;if(z<.01) continue;
      size_t index=size_t(y)*w+x; if(1/z<depth[index]-1e-7) continue;
      auto d=ray*z-cc;double a=dot(d,r)/hw,b=dot(d,u)/hh;
      if(std::abs(a)>1 || std::abs(b)>1) continue;
      if(texture==2 && a*a+b*b>1) continue;
      if(texture==3 && std::hypot(a,b)>.99) continue;
      if(texture==4) {
        const double radius=hw/hh,cap=std::max(0.,std::abs(b)-(1-radius))/radius;
        if(a*a+cap*cap>1)continue;
      }
      Color ink=color;
      if(texture==1) {
        int tx=std::clamp(int((a+1)*ArmorMarking::width/2),0,ArmorMarking::width-1);
        int ty=std::clamp(int((1-b)*ArmorMarking::height/2),0,ArmorMarking::height-1);
        bool white=marking->ink(tx,ty);
        ink=white ? Color{240,240,240}:Color{12,12,12};
      }
      depth[index]=float(1/z);
      for(int channel=0;channel<3;++channel) pixels[index*3+channel]=ink[channel];
    }
  }

};
}
std::shared_ptr<const Image> render_camera(const Snapshot& snapshot,const Pose& pose,
    const CameraCalibration& k,const ImageConfig& config,uint64_t seed) {
  std::shared_ptr<Image> image;
  if(config.renderer!="cpu") {
#ifdef RM_SIM_CAMERA_GPU
    thread_local std::string gpu_failure;
    if(gpu_failure.empty()) {
      try {image=camera_detail::render_gpu(snapshot,pose,k,config);}
      catch(const std::exception& error) {
        if(config.renderer=="gpu")throw;
        gpu_failure=error.what();
        std::cerr<<"[camera] GPU unavailable; using native CPU renderer: "<<gpu_failure<<'\n';
      }
    }else ensure(config.renderer!="gpu",gpu_failure.c_str());
#else
    ensure(config.renderer!="gpu","GPU camera was not built; enable RM_SIM_CAMERA_GPU with EGL/OpenGL");
#endif
  }
  if(!image) {
    Raster raster(k,pose);
    camera_detail::planes(snapshot,pose,config,[&](Vec3 c,Vec3 right,Vec3 up,double hw,double hh,
        Color color,int texture,const ArmorMarking* marking){raster.plane(c,right,up,hw,hh,color,texture,marking);});
    image=std::make_shared<Image>();image->width=k.width;image->height=k.height;
    image->bgr=std::move(raster.pixels);
  }
  if(config.noise_sigma>0) {
    std::mt19937_64 rng(seed);std::normal_distribution<double> noise(0,config.noise_sigma);
    for(auto& pixel:image->bgr)pixel=uint8_t(std::clamp(std::round(pixel+noise(rng)),0.,255.));
  }
  return image;
}
} // namespace rm_sim
