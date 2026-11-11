#pragma once

#include <QColor>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "rm_sim/world.hpp"
#include "rm_sim/armor_markings.hpp"

// Floating receiving surfaces only; geometry and all motion come from world snapshots.
namespace rm_sim::visual {
struct Face {
  std::vector<Vec3> vertices;
  QColor color;
  bool emissive = false;
};
class Model {
 public:
  std::vector<Face> faces;
  void polygon(std::vector<Vec3> v, QColor color, bool emissive = false) {
    faces.push_back({std::move(v), color, emissive});
  }
  void box(Vec3 c, Vec3 x, Vec3 y, Vec3 z, QColor color) {
    const Vec3 v[] = {c - x - y - z, c + x - y - z, c + x + y - z, c - x + y - z,
                      c - x - y + z, c + x - y + z, c + x + y + z, c - x + y + z};
    for (auto f : std::vector<std::vector<int>>{
             {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}})
      polygon({v[f[0]], v[f[1]], v[f[2]], v[f[3]]}, color);
  }
  void cylinder(Vec3 a, Vec3 b, double radius, QColor color, int segments = 16,
                double end_radius = -1) {
    if (norm(b - a) < 1e-8) return;
    auto axis = unit(b - a);
    auto u = unit(cross(axis, std::abs(axis.z) < .9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0}));
    auto v = cross(axis, u);
    if (end_radius < 0) end_radius = radius;
    std::vector<Vec3> bottom, top;
    for (int i = 0; i < segments; ++i) {
      auto r = u * std::cos(2 * pi * i / segments) + v * std::sin(2 * pi * i / segments);
      bottom.push_back(a + r * radius);
      top.push_back(b + r * end_radius);
    }
    for (int i = 0; i < segments; ++i) {
      int j = (i + 1) % segments;
      polygon({bottom[i], bottom[j], top[j], top[i]}, color);
    }
    polygon(top, color);
    std::reverse(bottom.begin(), bottom.end());
    polygon(bottom, color);
  }
  void strip(Vec3 a, Vec3 b, Vec3 normal, double width, QColor color, bool light = false) {
    if (norm(b - a) < 1e-8) return;
    auto side = unit(cross(normal, b - a)) * (width / 2);
    polygon({a - side, b - side, b + side, a + side}, color, light);
  }
  void ring(Vec3 c, Vec3 u, Vec3 v, double radius, double width, QColor color) {
    for (int i = 0; i < 40; ++i) {
      auto a = u * std::cos(i * pi / 20) + v * std::sin(i * pi / 20);
      auto b = u * std::cos((i + 1) * pi / 20) + v * std::sin((i + 1) * pi / 20);
      polygon({c + a * radius, c + b * radius, c + b * (radius - width), c + a * (radius - width)},
              color, true);
    }
  }
  void armor(const Surface & s, QColor team, const ArmorMarking& marking) {
    const auto r = s.right, u = s.up, n = s.normal;
    box(s.center - n * .012, r * (s.width / 2), u * (s.height / 2), n * .012, QColor("#252b33"));
    // A dark impact panel with two narrow team-colored light bars.
    auto c = s.center + n * .001;
    polygon(
        {c - r * s.width * .34 - u * s.height * .37, c + r * s.width * .34 - u * s.height * .37,
         c + r * s.width * .34 + u * s.height * .37, c - r * s.width * .34 + u * s.height * .37},
        QColor("#3b424c"));
    for (double side : {-1., 1.}) {
      auto a = c + r * (s.width * .43 * side);
      strip(a - u * s.height * .39, a + u * s.height * .39, n, s.width * .085, team, true);
      strip(a + n * .0005 - u * s.height * .31, a + n * .0005 + u * s.height * .31, n,
            s.width * .025, team.lighter(165), true);
    }
    // The same RM2026 mask is sampled by the synthetic camera.
    c = c + n * .002;
    auto point=[&](int x,int y){return c+r*((double(x)/ArmorMarking::width-.5)*ArmorMarking::physical_width)
      +u*((.5-double(y)/ArmorMarking::height)*ArmorMarking::physical_height);};
    for(size_t i=0;i<marking.rectangle_count;++i) {
      const auto& p=marking.rectangles[i];
      polygon({point(p.left,p.top),point(p.right,p.top),point(p.right,p.bottom),point(p.left,p.bottom)},
        QColor("#e2e8f0"),true);
    }
  }
  void target(const TargetState & t, QColor team) {
    for (const auto& s:t.surfaces) {
      if (t.kind==TargetKind::car || t.kind==TargetKind::outpost) {
        armor(s,team,armor_marking(t));
        continue;
      }
      const auto n=s.normal;
      const auto light=s.active ? team:QColor("#66717d");
      cylinder(s.center-n*.005,s.center+n*.003,s.disk_radius,QColor("#282d34"),40);
      for(int i=1;i<=10;++i)
        ring(s.center+n*.004,s.right,s.up,s.disk_radius*i/10,.0018,QColor("#727a84"));
      ring(s.center+n*.005,s.right,s.up,s.disk_radius,.006,light);
    }
  }
  void impact_marks(const Snapshot& state,const std::vector<Hit>& hits) {
    for(const auto& hit:hits) {
      const double age=state.time-hit.time;
      if(age<0 || age>=5 || !hit.plate_point)continue;
      for(const auto& target:state.targets) {
        if(target.id!=hit.target_id)continue;
        for(const auto& surface:target.surfaces) {
          if(surface.plate_id!=hit.plate_id)continue;
          const auto local=*hit.plate_point;
          // Put the decal just above the panel/marking geometry. It shares the
          // scene depth buffer, so the plate hides it when its back is visible.
          const auto center=surface.center+surface.right*local.x+surface.up*local.y+
            surface.normal*.006;
          std::vector<Vec3> disk;
          for(int i=0;i<16;++i) {
            const double angle=2*pi*i/16;
            disk.push_back(center+(surface.right*std::cos(angle)+surface.up*std::sin(angle))*.005);
          }
          polygon(std::move(disk),QColor("#22c55e"),true);
        }
      }
    }
  }
};
}  // namespace rm_sim::visual
