#include "rm_sim/image_io.hpp"
#include <png.h>
#include <iomanip>
#include <sstream>

namespace rm_sim {
ImageArchive::ImageArchive(const std::filesystem::path& trace_path)
 :directory_(std::filesystem::absolute(trace_path).string()+".images") {}
std::string ImageArchive::write(const Image& image) {
  ensure(image.width>0 && image.height>0 && image.bgr.size()==size_t(image.width)*image.height*3,"Invalid BGR image");
  // Content-addressed frames avoid duplicating stationary images, and make paths reproducible.
  uint64_t hash=14695981039346656037ULL;
  for(auto value:{image.width,image.height}) for(int shift=0;shift<32;shift+=8) {
    hash^=(unsigned(value)>>shift)&255;hash*=1099511628211ULL;
  }
  for(auto byte:image.bgr) { hash^=byte;hash*=1099511628211ULL; }
  std::ostringstream name; name<<std::hex<<std::setfill('0')<<std::setw(16)<<hash<<".png";
  std::filesystem::create_directories(directory_);
  auto path=directory_/name.str();
  if(!std::filesystem::exists(path)) {
    png_image png{};png.version=PNG_IMAGE_VERSION;
    png.width=image.width;png.height=image.height;png.format=PNG_FORMAT_BGR;
    png.flags=PNG_IMAGE_FLAG_FAST;
    auto temporary=path.string()+".tmp";
    if(!png_image_write_to_file(&png,temporary.c_str(),0,image.bgr.data(),0,nullptr)) {
      std::string error=png.message;png_image_free(&png);
      std::filesystem::remove(temporary);
      throw std::runtime_error("Cannot write camera PNG: "+error);
    }
    png_image_free(&png);
    std::filesystem::rename(temporary,path);
  }
  return (directory_.filename()/path.filename()).generic_string();
}
} // namespace rm_sim
