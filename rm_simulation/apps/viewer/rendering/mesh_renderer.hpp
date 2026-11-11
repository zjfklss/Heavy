#pragma once

#include <QImage>
#include <limits>

#include "models.hpp"

namespace rm_sim::visual {
// Small CPU depth-buffered triangle renderer. No graphics context is needed offscreen.
inline QImage render(const Model & model, int width, int height, Vec3 eye, Vec3 forward, Vec3 right,
                     Vec3 up, double scale) {
  constexpr int sampling = 2;
  width *= sampling;
  height *= sampling;
  scale *= sampling;
  QImage image(width, height, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  std::vector<float> depth(size_t(width) * height, 0);
  struct Vertex {
    double x, y, inverse_z;
  };
  auto project = [&](Vec3 v) {
    auto d = v - eye;
    double z = dot(d, forward);
    return Vertex{width * .5 + scale * dot(d, right) / z, height * .5 - scale * dot(d, up) / z,
                  1 / z};
  };
  auto edge = [](Vertex a, Vertex b, double x, double y) {
    return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
  };
  auto triangle = [&](Vertex a, Vertex b, Vertex c, QRgb color) {
    double area = edge(a, b, c.x, c.y);
    if (std::abs(area) < 1e-8) return;
    int left = int(std::max(0., std::floor(std::min({a.x, b.x, c.x}))));
    int top = int(std::max(0., std::floor(std::min({a.y, b.y, c.y}))));
    int end_x = int(std::min(double(width - 1), std::ceil(std::max({a.x, b.x, c.x}))));
    int end_y = int(std::min(double(height - 1), std::ceil(std::max({a.y, b.y, c.y}))));
    for (int y = top; y <= end_y; ++y) {
      auto pixels = reinterpret_cast<QRgb *>(image.scanLine(y));
      for (int x = left; x <= end_x; ++x) {
        double wa = edge(b, c, x + .5, y + .5) / area;
        double wb = edge(c, a, x + .5, y + .5) / area;
        double wc = 1 - wa - wb;
        if (wa < 0 || wb < 0 || wc < 0) continue;
        double iz = wa * a.inverse_z + wb * b.inverse_z + wc * c.inverse_z;
        auto & previous = depth[size_t(y) * width + x];
        if (iz > previous) {
          previous = float(iz);
          pixels[x] = color;
        }
      }
    }
  };
  const auto light = unit(Vec3{-1, -2, 4});
  for (const auto & face : model.faces) {
    if (face.vertices.size() < 3) continue;
    auto normal = cross(face.vertices[1] - face.vertices[0], face.vertices[2] - face.vertices[0]);
    if (norm(normal) < 1e-10) continue;
    const double intensity = face.emissive ? 1.0 : .70 + .30 * std::abs(dot(unit(normal), light));
    QRgb color = qRgb(int(face.color.red() * intensity), int(face.color.green() * intensity),
                      int(face.color.blue() * intensity));
    // Clip to the near plane before perspective division.
    std::vector<Vec3> clipped;
    for (size_t i = 0; i < face.vertices.size(); ++i) {
      auto a = face.vertices[i], b = face.vertices[(i + 1) % face.vertices.size()];
      double za = dot(a - eye, forward), zb = dot(b - eye, forward);
      if (za >= .05) clipped.push_back(a);
      if ((za >= .05) != (zb >= .05)) clipped.push_back(a + (b - a) * ((.05 - za) / (zb - za)));
    }
    if (clipped.size() < 3) continue;
    auto first = project(clipped[0]);
    for (size_t i = 1; i + 1 < clipped.size(); ++i)
      triangle(first, project(clipped[i]), project(clipped[i + 1]), color);
  }
  return image.scaled(width / sampling, height / sampling, Qt::IgnoreAspectRatio,
                      Qt::SmoothTransformation);
}
}  // namespace rm_sim::visual
