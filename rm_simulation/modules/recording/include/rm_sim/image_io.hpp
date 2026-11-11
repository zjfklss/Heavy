#pragma once
#include "rm_sim/image.hpp"
#include <filesystem>

namespace rm_sim {
// PNG sidecars; the world/sensor/runner libraries do not depend on libpng.
class ImageArchive {
 public:
  explicit ImageArchive(const std::filesystem::path& trace_path);
  std::string write(const Image& image);
 private:
  std::filesystem::path directory_;
};
} // namespace rm_sim
