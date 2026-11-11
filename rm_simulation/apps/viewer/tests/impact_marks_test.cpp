#include "../rendering/mesh_renderer.hpp"
#include <iostream>
#include <stdexcept>
using namespace rm_sim;
void check(bool value,const char* message) {
  if(!value)throw std::runtime_error(message);
}
int main() {
  try {
    Surface plate;plate.target_id="robot";plate.plate_id=2;plate.center={3,0,.3};
    plate.normal={-1,0,0};plate.right={0,-1,0};plate.up={0,0,1};plate.width=.135;plate.height=.125;
    TargetState target;target.id="robot";target.kind=TargetKind::car;target.surfaces={plate};
    Snapshot state;state.time=1;state.targets={target};
    Hit hit;hit.time=1;hit.target_id="robot";hit.plate_id=2;hit.plate_point=Vec3{.02,.025,0};
    hit.point=plate.center+plate.right*.02+plate.up*.025;
    visual::Model first;first.impact_marks(state,{hit});
    check(first.faces.size()==1,"contact mark missing");
    auto center=[](const visual::Model& model){Vec3 c;for(auto v:model.faces[0].vertices)c=c+v;return c*(1./16);};
    check(norm(center(first)-(hit.point+plate.normal*.006))<1e-10,"contact position changed");
    // Follow both translation and a 90-degree rotation while retaining the same plate ID.
    state.time=5.999;auto& moved=state.targets[0].surfaces[0];
    moved.center={4,1,.8};moved.normal={0,-1,0};moved.right={1,0,0};
    visual::Model later;later.impact_marks(state,{hit});
    check(later.faces.size()==1,"mark disappeared before five seconds");
    check(norm(center(later)-Vec3{4.02,.994,.825})<1e-10,"mark did not follow plate motion");
    state.time=6;visual::Model expired;expired.impact_marks(state,{hit});
    check(expired.faces.empty(),"mark survived five seconds");
    state.time=.999;visual::Model early;early.impact_marks(state,{hit});
    check(early.faces.empty(),"mark appeared before impact");
    state.time=1;state.targets={target};
    auto other=hit;other.target_id="other";other.plate_id=2;
    visual::Model wrong;wrong.impact_marks(state,{other});
    check(wrong.faces.empty(),"mark attached to a different target");
    other=hit;other.plate_id=3;wrong.impact_marks(state,{other});
    check(wrong.faces.empty(),"mark attached to a different plate");
    visual::Model scene;scene.target(target,QColor("#e53945"));scene.impact_marks(state,{hit});
    auto green_pixels=[](const QImage& image){int count=0;for(int y=0;y<image.height();++y)
      for(int x=0;x<image.width();++x){auto c=image.pixelColor(x,y);count+=c.alpha()>0 && c.green()>120 && c.green()>c.red()*1.5 && c.green()>c.blue()*1.5;}return count;};
    check(green_pixels(visual::render(scene,600,400,{0,0,.3},{1,0,0},{0,-1,0},{0,0,1},1100))>0,
      "front-facing contact mark is invisible");
    check(green_pixels(visual::render(scene,600,400,{6,0,.3},{-1,0,0},{0,1,0},{0,0,1},1100))==0,
      "contact mark shows through the plate back");
    std::cout<<"Impact attachment, five-second lifetime and depth occlusion passed\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
