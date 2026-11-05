#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>
#include <cstring>
#include <filesystem>
#include <fstream>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid, double fps,
  const HikRobotSettings & settings)
: exposure_us_(exposure_ms * 1e3), gain_(gain), fps_(fps), settings_(settings),
  daemon_quit_(false), queue_(1), vid_(-1), pid_(-1)
{
  if (!vid_pid.empty()) set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  if (!queue_.try_pop_for(data, std::chrono::milliseconds(200))) {
    img.release();
    timestamp = std::chrono::steady_clock::now();
    return;
  }

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  MV_CC_DEVICE_INFO * selected = nullptr;
  for (unsigned int index = 0; index < device_list.nDeviceNum; ++index) {
    auto * device = device_list.pDeviceInfo[index];
    if (settings_.serial == reinterpret_cast<const char *>(device->SpecialInfo.stUsb3VInfo.chSerialNumber)) {
      selected = device;
      break;
    }
  }
  if (!selected) {
    tools::logger()->warn("Camera serial {} not found", settings_.serial);
    return;
  }
  for (const auto & entry : std::filesystem::directory_iterator("/sys/bus/usb/devices")) {
    std::ifstream serial_file(entry.path() / "serial");
    std::string serial;
    if (!(serial_file >> serial) || serial != settings_.serial) continue;
    std::ifstream speed_file(entry.path() / "speed");
    double speed = 0;
    if (!(speed_file >> speed)) continue;
    if (speed < 5000)
      tools::logger()->warn("[HikRobot] USB link {} Mbit/s; full-resolution 165 FPS requires USB3", speed);
    else
      tools::logger()->info("[HikRobot] USB link {} Mbit/s", speed);
  }
  ret = MV_CC_CreateHandle(&handle_, selected);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  // Clear old crop offsets before increasing the image dimensions.
  if (settings_.offset_x >= 0 && !set_int_value("OffsetX", 0)) return;
  if (settings_.offset_y >= 0 && !set_int_value("OffsetY", 0)) return;
  if (settings_.image_width > 0 && !set_int_value("Width", settings_.image_width)) return;
  if (settings_.image_height > 0 && !set_int_value("Height", settings_.image_height)) return;
  if (settings_.offset_x > 0 && !set_int_value("OffsetX", settings_.offset_x)) return;
  if (settings_.offset_y > 0 && !set_int_value("OffsetY", settings_.offset_y)) return;
  if (settings_.trigger_enable) {
    ret = MV_CC_SetEnumValue(
      handle_, "TriggerMode", *settings_.trigger_enable ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
    if (ret != MV_OK) {
      tools::logger()->warn("MV_CC_SetEnumValue(\"TriggerMode\") failed: {:#x}", ret);
      return;
    }
  }
  if (settings_.frame_rate_enable &&
      !set_bool_value("AcquisitionFrameRateEnable", *settings_.frame_rate_enable)) return;

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("PixelFormat", PixelType_Gvsp_BayerRG8);
  ret = MV_CC_SetImageNodeNum(handle_, settings_.image_node_num);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetImageNodeNum failed: {:#x}", ret);
    return;
  }
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  ret = MV_CC_SetFrameRate(handle_, fps_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFrameRate({}) failed: {:#x}", fps_, ret);
    return;
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw{};
    auto last_frame = std::chrono::steady_clock::now();
    bool timeout_reported = false;

    while (!capture_quit_) {
      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret == MV_E_NODATA && std::chrono::steady_clock::now() - last_frame < 1s) {
        if (!timeout_reported) {
          tools::logger()->warn("[HikRobot] Frame timeout; retrying without restarting camera");
          timeout_reported = true;
        }
        continue;
      }
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      last_frame = timestamp;
      timeout_reported = false;
      ++frames_captured_;
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8UC3);

      MV_CC_PIXEL_CONVERT_PARAM cvt_param{};
      cvt_param.nWidth = raw.stFrameInfo.nWidth;
      cvt_param.nHeight = raw.stFrameInfo.nHeight;

      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

      cvt_param.pDstBuffer = img.data;
      cvt_param.nDstBufferSize = img.total() * img.elemSize();
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      // Let MVS handle the camera's actual source format; SP consumes BGR8.
      ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_ConvertPixelType failed: {:#x}", ret);
        MV_CC_FreeImageBuffer(handle_, &raw);
        break;
      }

      queue_.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();
  if (!handle_) return;

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
  }
  handle_ = nullptr;
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

bool HikRobot::set_int_value(const std::string & name, int value)
{
  auto ret = MV_CC_SetIntValueEx(handle_, name.c_str(), value);
  if (ret == MV_OK) return true;
  tools::logger()->warn("MV_CC_SetIntValueEx(\"{}\", {}) failed: {:#x}", name, value, ret);
  return false;
}

bool HikRobot::set_bool_value(const std::string & name, bool value)
{
  auto ret = MV_CC_SetBoolValue(handle_, name.c_str(), value);
  if (ret == MV_OK) return true;
  tools::logger()->warn("MV_CC_SetBoolValue(\"{}\", {}) failed: {:#x}", name, value, ret);
  return false;
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
