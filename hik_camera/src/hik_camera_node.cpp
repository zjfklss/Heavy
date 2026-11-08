/// 1duan
#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "jlcv_interfaces/msg/game_status.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <string>
#include <vector>


namespace hik_camera
{
namespace
{
const char * hikErrName(int ret)
{
  switch (static_cast<unsigned int>(ret))
  {
    case MV_OK: return "MV_OK";
    case MV_E_HANDLE: return "MV_E_HANDLE(invalid handle)";
    case MV_E_SUPPORT: return "MV_E_SUPPORT(unsupported)";
    case MV_E_BUFOVER: return "MV_E_BUFOVER";
    case MV_E_CALLORDER: return "MV_E_CALLORDER(wrong call order / not ready)";
    case MV_E_PARAMETER: return "MV_E_PARAMETER";
    case MV_E_RESOURCE: return "MV_E_RESOURCE";
    case MV_E_NODATA: return "MV_E_NODATA";
    case MV_E_ACCESS_DENIED: return "MV_E_ACCESS_DENIED(no permission / occupied?)";
    case MV_E_BUSY: return "MV_E_BUSY(device busy or occupied)";
    case MV_E_USB_DEVICE: return "MV_E_USB_DEVICE";
    case MV_E_USB_BANDWIDTH: return "MV_E_USB_BANDWIDTH";
    case MV_E_USB_DRIVER: return "MV_E_USB_DRIVER";
    default: return "unknown";
  }
}
}  // namespace

class HikCameraNode : public rclcpp::Node
{
public:

      explicit HikCameraNode(const rclcpp::NodeOptions& options) : Node("camera_node", options)
    {

      // 初始化指针为 nullptr
      camera_handle_ = nullptr;
      camera_handle_long = nullptr;
      camera_handle_short = nullptr;
      device_info_ = nullptr;
      device_info_short_ = nullptr;

      RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");
      
      MV_CC_DEVICE_INFO_LIST device_list;
      // enum device
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      frame_id_short = "camera_optical_frame";
      frame_id_long = "camera_optical_frame1";
      RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

      while (device_list.nDeviceNum == 0 && rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      }

      //
    
      camera_num = device_list.nDeviceNum;
      // ================================================================================================================




      // 必须先存成 std::string：declare_parameter 返回临时对象，.c_str() 会悬空
      // const char* serial_chang = this->declare_parameter("camera_serial", "DA183077").c_str();
      // const char* serial_duan = this->declare_parameter("camera_serial1", "DA071003").c_str();
      const std::string serial_chang = this->declare_parameter("camera_serial1", "DA9648617");
      const std::string serial_duan = this->declare_parameter("camera_serial", "DA1830770");
      
      std::cout<<"serial_chang "<<serial_chang<<"\nserial_duan "<<serial_duan<<std::endl;
      if(camera_num==2)
      {
        auto serial0 = device_list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chSerialNumber;
        auto serial1 = device_list.pDeviceInfo[1]->SpecialInfo.stUsb3VInfo.chSerialNumber;

        std::cout<<"serial0 "<<serial0<<"\nserial1 "<<serial1<<std::endl;
        // short-only: 只创建并打开短焦，长焦完全不碰
        if(strcmp(reinterpret_cast<char*>(serial0),serial_duan.c_str())==0)
        {
          device_info_short_ = device_list.pDeviceInfo[0];
        }
        else if(strcmp(reinterpret_cast<char*>(serial1),serial_duan.c_str())==0)
        {
          device_info_short_ = device_list.pDeviceInfo[1];
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Short camera serial not found, fallback to device 0");
          device_info_short_ = device_list.pDeviceInfo[0];
        }
        camera_handle_long = nullptr;
        if (!createAndOpen(camera_handle_short, device_info_short_, "short"))
        {
          throw std::runtime_error("Failed to open short camera (busy/occupied or OpenDevice failed)");
        }
      }
      else if(camera_num==1)
      {
        device_info_ = device_list.pDeviceInfo[0];
        if (!createAndOpen(camera_handle_, device_info_, "camera"))
        {
          throw std::runtime_error("Failed to open camera (busy/occupied or OpenDevice failed)");
        }
      }
      

      // MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
      RCLCPP_INFO(this->get_logger(), "Camera Number is %d.", camera_num);

      // PixelFormat / SDK 缓存节点数 必须早于 StartGrabbing；
      // 曝光、增益、分辨率、帧率也必须在开始取流之前设置，否则海康 SDK 会静默失败（返回值仍可能是 MV_OK）
      if (!configureStream())
      {
        auto closeOnFail = [](void *& handle) {
          if (handle)
          {
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(&handle);
            handle = nullptr;
          }
        };
        closeOnFail(camera_handle_);
        closeOnFail(camera_handle_long);
        closeOnFail(camera_handle_short);
        throw std::runtime_error(
          "Camera stream configure failed (PixelFormat). "
          "Close MVS Client / other camera_node, replug USB3, then relaunch.");
      }
      declareParameters();

      if(camera_num == 2){
        // short-only: 长焦不初始化、不取流
        if(camera_handle_short){
          MV_CC_GetImageInfo(camera_handle_short, &img_info_);
          if (img_info_.nWidthValue > 0 && img_info_.nWidthValue <= 5000 &&
              img_info_.nHeightValue > 0 && img_info_.nHeightValue <= 5000) {
            MV_CC_StartGrabbing(camera_handle_short);
            convert_param_duan.nWidth = img_info_.nWidthValue;
            convert_param_duan.nHeight = img_info_.nHeightValue;
            convert_param_duan.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid image size for short camera: %dx%d", 
                        img_info_.nWidthValue, img_info_.nHeightValue);
          }
        }
     
      }
      else if(camera_num == 1 && camera_handle_){
        MV_CC_GetImageInfo(camera_handle_, &img_info_);
        nRet = MV_CC_StartGrabbing(camera_handle_);
        if (nRet != MV_OK)
        {
          RCLCPP_FATAL(this->get_logger(), "StartGrabbing failed: [%x] %s", nRet, hikErrName(nRet));
          auto closeOnFail = [](void *& handle) {
            if (handle)
            {
              MV_CC_CloseDevice(handle);
              MV_CC_DestroyHandle(&handle);
              handle = nullptr;
            }
          };
          closeOnFail(camera_handle_);
          throw std::runtime_error("StartGrabbing failed");
        }
        convert_param_.nWidth = img_info_.nWidthValue;
        convert_param_.nHeight = img_info_.nHeightValue;
      }
      // MV_CC_GetImageInfo(camera_handle_, &img_info_);
      // image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
      // MV_CC_StartGrabbing(camera_handle_);

      // Init convert param 
      // convert_param_.nWidth = img_info_.nWidthValue;
      // convert_param_.nHeight = img_info_.nHeightValue;
      convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;  // PixelType_Gvsp_RGB8_Packed

     // 只声明参数，但不接收返回值，这样既防止 launch 文件报错，又消除了未使用变量的警告
     this->declare_parameter("use_sensor_data_qos", true);
     auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                        .best_effort()
                        .durability_volatile();
     // camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
     image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos_profile);
     camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos_profile);

      // Load camera info
      camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
      camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
      camera_info_url = this->declare_parameter("camera_info_url", "package://vision_bringup/config/camera_info.yaml");
      camera_info_url1 = this->declare_parameter("camera_info_url1", "package://vision_bringup/config/camera_info1.yaml");
      if (camera_info_manager_->validateURL(camera_info_url))
      {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_short_msg_ = camera_info_manager_->getCameraInfo();
      }else{
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      }
      
      if (camera_info_manager_->validateURL(camera_info_url1))
      {
        camera_info_manager_->loadCameraInfo(camera_info_url1);
        camera_info_long_msg_ = camera_info_manager_->getCameraInfo();
      }else{
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url1.c_str());
      }
      

      params_callback_handle_ = this->add_on_set_parameters_callback(
          std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

      game_status_sub_ = this->create_subscription<jlcv_interfaces::msg::GameStatus>(
        "/game_status", 10,
        [this](const jlcv_interfaces::msg::GameStatus::SharedPtr msg) {
          latest_game_status_ = *msg;
        }
      );
      
      if(camera_num == 2){
        // short-only: 固定短焦
        camera_status_ = 0;
        MV_CC_StopGrabbing(camera_handle_short);
        frame_id = frame_id_short;
        camera_handle_ = camera_handle_short;
        convert_param_.nWidth = convert_param_duan.nWidth;
        convert_param_.nHeight = convert_param_duan.nHeight;
        MV_CC_StartGrabbing(camera_handle_short);
        RCLCPP_INFO(this->get_logger(), "Short-only mode: long camera disabled");
      }
      else if(camera_num == 1 && camera_handle_){
        // 单相机：固定短焦 frame，避免下游 MessageFilter 因空 frame_id 丢包
        camera_status_ = 0;
        frame_id = frame_id_short;
        convert_param_.nWidth = img_info_.nWidthValue;
        convert_param_.nHeight = img_info_.nHeightValue;
        RCLCPP_INFO(this->get_logger(), "Single camera mode: frame_id=%s", frame_id.c_str());
      }

      // ===========================================================================================================================
      
      capture_thread_ = std::thread{ [this]() -> void {
        if (camera_num == 1){
          RCLCPP_INFO(this->get_logger(), "Single camera mode: long/short switching disabled.");
        }
        MV_FRAME_OUT out_frame;
        RCLCPP_INFO(this->get_logger(), "Publishing image!");
        
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待系统稳定，避免刚启动时抓取失败
        MV_CC_ClearImageBuffer(camera_handle_);                // 丢掉睡眠期间堆积的旧帧

        auto rate_begin = std::chrono::steady_clock::now();
        int rate_count = 0;

        while (rclcpp::ok())
        {
          void* handle = nullptr;
          {
            // 只有切换长短焦时才需要持锁；取流阶段不持锁，避免阻塞 ros2 param 回调
            std::lock_guard<std::mutex> lock(camera_mutex_);

            // short-only: 忽略电控 camera_status，始终保持短焦
            if (false)
            {
              RCLCPP_INFO(this->get_logger(), "Camera status: %d -> %d", camera_status_,
                          latest_game_status_.camera_status);
              MV_CC_StopGrabbing(camera_handle_);
              camera_status_ = latest_game_status_.camera_status;
              if (camera_status_ == 0)
              {
                frame_id = frame_id_short;
                camera_handle_ = camera_handle_short;
                convert_param_.nWidth = convert_param_duan.nWidth;
                convert_param_.nHeight = convert_param_duan.nHeight;
                MV_CC_StartGrabbing(camera_handle_short);
              }
              else if (camera_status_ == 1)
              {
                frame_id = frame_id_long;
                camera_handle_ = camera_handle_long;
                convert_param_.nWidth = convert_param_chang.nWidth;
                convert_param_.nHeight = convert_param_chang.nHeight;
                MV_CC_StartGrabbing(camera_handle_long);
              }
              else
              {
                RCLCPP_ERROR(this->get_logger(), "Camera Status ERROR!");
              }
            }
            handle = camera_handle_;
          }

          if (handle == nullptr)
          {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Camera handle is null!");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }

          // 超时 200ms 足够；原来是 1000ms，取流一失败就是整秒的帧全丢
          nRet = MV_CC_GetImageBuffer(handle, &out_frame, 200);
          if (MV_OK != nRet)
          {
            ++fail_conut_;
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Get buffer failed! nRet: [%x] %s, consecutive: %d", nRet, hikErrName(nRet),
              fail_conut_);
            // 连续失败几次才重启取流；每帧 stop/start 会让帧率直接塌掉
            if (fail_conut_ % 5 == 0)
            {
              std::lock_guard<std::mutex> lock(camera_mutex_);
              MV_CC_StopGrabbing(handle);
              MV_CC_ClearImageBuffer(handle);
              MV_CC_StartGrabbing(handle);
            }
            if (fail_conut_ > 50)
            {
              RCLCPP_FATAL(
                this->get_logger(),
                "Camera failed after %d consecutive GetImageBuffer errors (last [%x] %s). "
                "Check USB3 / MVS Client / another process holding the camera.",
                fail_conut_, nRet, hikErrName(nRet));
              rclcpp::shutdown();
            }
            continue;
          }
          fail_conut_ = 0;

          const MV_FRAME_OUT_INFO_EX & info = out_frame.stFrameInfo;

          // 每帧新建消息（发布时移交所有权，下游节点可零拷贝）
          auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
          img_msg->header.stamp = this->now();
          img_msg->header.frame_id = frame_id;
          img_msg->width = info.nWidth;
          img_msg->height = info.nHeight;

          // 相机输出什么打包格式就发什么格式，不再每帧做一次 Bayer->RGB 转换
          const char* encoding = nullptr;
          unsigned int bytes_per_pixel = 0;
          switch (info.enPixelType)
          {
            case PixelType_Gvsp_BayerRG8:    encoding = "bayer_rg8"; bytes_per_pixel = 1; break;
            case PixelType_Gvsp_Mono8:       encoding = "mono8";     bytes_per_pixel = 1; break;
            case PixelType_Gvsp_RGB8_Packed: encoding = "rgb8";      bytes_per_pixel = 3; break;
            case PixelType_Gvsp_BGR8_Packed: encoding = "bgr8";      bytes_per_pixel = 3; break;
            default:                         encoding = nullptr;     bytes_per_pixel = 0; break;
          }

          if (encoding != nullptr)
          {
            img_msg->encoding = encoding;
            img_msg->step = info.nWidth * bytes_per_pixel;
            const size_t nbytes = static_cast<size_t>(img_msg->step) * info.nHeight;
            img_msg->data.resize(nbytes);
            std::memcpy(img_msg->data.data(), out_frame.pBufAddr,
                        std::min(nbytes, static_cast<size_t>(info.nFrameLen)));
          }
          else
          {
            // 兜底：Bayer10 / YUV 等非常规格式交给 SDK 转 RGB8
            img_msg->encoding = "rgb8";
            img_msg->step = info.nWidth * 3;
            img_msg->data.resize(static_cast<size_t>(img_msg->step) * info.nHeight);
            convert_param_.nWidth = info.nWidth;
            convert_param_.nHeight = info.nHeight;
            convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            convert_param_.pDstBuffer = img_msg->data.data();
            convert_param_.nDstBufferSize = static_cast<unsigned int>(img_msg->data.size());
            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = info.nFrameLen;
            convert_param_.enSrcPixelType = info.enPixelType;
            MV_CC_ConvertPixelType(handle, &convert_param_);
          }

          // 立刻归还 SDK 缓存：SDK 默认只有 1 个缓存节点，
          // 拿着 buffer 去转换/发布会让相机侧直接跳帧
          MV_CC_FreeImageBuffer(handle, &out_frame);

          // 发布camera_info
          camera_info_msg_ = (handle == camera_handle_long) ? camera_info_long_msg_ : camera_info_short_msg_;
          camera_info_msg_.header = img_msg->header;
          camera_info_pub_->publish(camera_info_msg_);

          // 发布图像（移交所有权）
          image_pub_->publish(std::move(img_msg));

          // 每 2 秒打印一次实测帧率，用来确认相机是否真的跑到了目标帧率
          ++rate_count;
          const double rate_elapsed =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - rate_begin).count();
          if (rate_elapsed > 2.0)
          {
            RCLCPP_INFO(this->get_logger(), "Camera fps: %.1f", rate_count / rate_elapsed);
            rate_count = 0;
            rate_begin = std::chrono::steady_clock::now();
          }
        }
      } };

    }
  
/**
 * @ brief HikCameraNode 类的析构函数，用于在对象销毁时进行资源清理工作
 * 
 * 该析构函数会执行以下操作：
 * 1. 等待图像捕获线程结束（如果线程可连接）。
 * 2. 停止相机的图像抓取操作，关闭相机设备并销毁相机句柄（如果相机句柄存在）。
 * 3. 输出日志信息，表明 HikCameraNode 已被销毁。
 */
  ~HikCameraNode() override
  {
    // 检查图像捕获线程是否可连接，如果可连接则等待线程结束
    if (capture_thread_.joinable())
    {
      capture_thread_.join();
    }

    // 关闭所有相机句柄
    auto closeCamera = [](void*& handle) {
      if (handle)
      {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(&handle);
        handle = nullptr;
      }
    };
    
    closeCamera(camera_handle_);
    closeCamera(camera_handle_long);
    closeCamera(camera_handle_short);

    // 输出日志信息，表明 HikCameraNode 已被销毁
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:

  bool createAndOpen(void *& handle, MV_CC_DEVICE_INFO * info, const char * tag)
  {
    if (info == nullptr)
    {
      RCLCPP_FATAL(this->get_logger(), "[%s] device info is null", tag);
      return false;
    }
    int ret = MV_CC_CreateHandle(&handle, info);
    if (ret != MV_OK)
    {
      RCLCPP_FATAL(this->get_logger(), "[%s] CreateHandle failed: [%x] %s", tag, ret, hikErrName(ret));
      handle = nullptr;
      return false;
    }
    ret = MV_CC_OpenDevice(handle);
    if (ret != MV_OK)
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "[%s] OpenDevice failed: [%x] %s. Close MVS Client / residual camera_node, check USB3.",
        tag, ret, hikErrName(ret));
      MV_CC_DestroyHandle(&handle);
      handle = nullptr;
      return false;
    }
    return true;
  }

  bool reopenDevice(void *& handle, MV_CC_DEVICE_INFO * info, const char * tag)
  {
    if (handle)
    {
      MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(&handle);
      handle = nullptr;
    }
    RCLCPP_WARN(this->get_logger(), "[%s] reopening device after MV_E_CALLORDER", tag);
    return createAndOpen(handle, info, tag);
  }

/**
 * @brief 在开始取流之前配置与"带宽/帧率"相关的设备参数
 *
 * - ImageNodeNum：SDK 内部图像缓存节点数，默认只有 1 个。
 *   一旦 host 侧处理慢于曝光周期，SDK 就没有空闲节点放新帧，表现为相机"跳帧"，
 *   海康官方注释也建议在性能一般的板子上加大这个值。
 * - PixelFormat：相机持久化在设备里的像素格式。默认 RGB8 时一帧 1440x1080 要 4.6MB，
 *   会被 USB3 带宽（DeviceLinkThroughputLimit）卡到 ~79fps；改成 BayerRG8 后一帧 1.5MB，
 *   实测可以跑满 165fps，同时把 DDS 流量降到 1/3。
 * - BayerCvtQuality：只影响 host 端 MV_CC_ConvertPixelType 的插值算法，取流前设一次即可。
 *
 * @return false 若 PixelFormat 设置失败（含 CALLORDER 重开后仍失败）；调用方不得 StartGrabbing / Publishing。
 */
  bool configureStream()
  {
    const std::string pixel_format = this->declare_parameter("pixel_format", std::string("bayer_rg8"));
    const int image_node_num = static_cast<int>(this->declare_parameter("image_node_num", 8));

    unsigned int want_format = 0;
    if (pixel_format == "bayer_rg8")
    {
      want_format = PixelType_Gvsp_BayerRG8;
    }
    else if (pixel_format == "mono8")
    {
      want_format = PixelType_Gvsp_Mono8;
    }
    else if (pixel_format == "rgb8")
    {
      want_format = PixelType_Gvsp_RGB8_Packed;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unknown pixel_format \"%s\", keep camera default.",
                  pixel_format.c_str());
    }

    auto applyOnce = [this, want_format, image_node_num](void * handle, const char * tag) -> int {
      if (handle == nullptr)
      {
        return MV_OK;
      }

      int ret = MV_CC_SetImageNodeNum(handle, static_cast<unsigned int>(image_node_num));
      if (ret != MV_OK)
      {
        RCLCPP_WARN(this->get_logger(), "[%s] SetImageNodeNum(%d) failed: [%x] %s", tag,
                    image_node_num, ret, hikErrName(ret));
        // ImageNodeNum 失败通常仍可取流；PixelFormat 才是硬失败
      }

      if (want_format != 0)
      {
        ret = MV_CC_SetEnumValue(handle, "PixelFormat", want_format);
        if (ret != MV_OK)
        {
          return ret;
        }
      }

      MVCC_ENUMVALUE enum_value{};
      if (MV_CC_GetEnumValue(handle, "PixelFormat", &enum_value) == MV_OK)
      {
        RCLCPP_INFO(this->get_logger(), "[%s] PixelFormat = 0x%x (want 0x%x)", tag,
                    enum_value.nCurValue, want_format);
        if (want_format != 0 && enum_value.nCurValue != want_format)
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "[%s] PixelFormat mismatch after set: got 0x%x want 0x%x (format unsupported?)",
            tag, enum_value.nCurValue, want_format);
          return MV_E_SUPPORT;
        }
      }

      MV_CC_SetBayerCvtQuality(handle, 1);
      return MV_OK;
    };

    auto setup = [this, &applyOnce, want_format](void *& handle, MV_CC_DEVICE_INFO * info,
                                                 const char * tag) -> bool {
      if (handle == nullptr)
      {
        return true;
      }

      int ret = applyOnce(handle, tag);
      if (ret == static_cast<int>(MV_E_CALLORDER) && info != nullptr)
      {
        if (!reopenDevice(handle, info, tag))
        {
          return false;
        }
        ret = applyOnce(handle, tag);
      }

      if (want_format != 0 && ret != MV_OK)
      {
        RCLCPP_FATAL(
          this->get_logger(),
          "[%s] Set PixelFormat 0x%x failed: [%x] %s. "
          "Will NOT start grabbing. Close MVS / residual nodes, or try another pixel_format.",
          tag, want_format, ret, hikErrName(ret));
        return false;
      }
      return true;
    };

    bool ok = true;
    ok = setup(camera_handle_long, nullptr, "long") && ok;
    ok = setup(camera_handle_short, device_info_short_, "short") && ok;
    ok = setup(camera_handle_, device_info_, "camera") && ok;
    return ok;
  }

/**
 * @ brief 声明并设置相机的各项参数
 * 
 * 该函数用于从 ROS 参数服务器获取相机的各项参数，并将这些参数设置到相机设备中。
 * 同时，会获取相机参数的取值范围，并将其作为参数描述符的一部分。
 */

  void declareParameters()
  {
    // 创建参数描述符对象，用于描述参数的属性
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    // 定义一个结构体，用于存储相机的浮点型参数值
    MVCC_FLOATVALUE f_value;
    // 为参数描述符的整数范围数组分配空间，用于存储参数的取值范围
    param_desc.integer_range.resize(1);
    // 设置参数取值范围的步长为 1
    param_desc.integer_range[0].step = 1;


    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    if(camera_num==2)
    {
      if(camera_handle_long){
        MV_CC_GetFloatValue(camera_handle_long, "ExposureTime", &f_value);
        MV_CC_GetFloatValue(camera_handle_short, "ExposureTime", &f_value);
        param_desc.integer_range[0].from_value = f_value.fMin;
        param_desc.integer_range[0].to_value = f_value.fMax;
        double exposure_time = this->declare_parameter("exposure_time", 7500.0, param_desc);
        MV_CC_SetFloatValue(camera_handle_long, "ExposureTime", exposure_time);
        RCLCPP_INFO(this->get_logger(), "Exposure_chang time: %f", exposure_time);

      }
      if (camera_handle_short) {
        MV_CC_GetFloatValue(camera_handle_short, "ExposureTime", &f_value);
        param_desc.integer_range[0].from_value = f_value.fMin;
        param_desc.integer_range[0].to_value = f_value.fMax;
        double exposure_time = this->declare_parameter("exposure_time1", 7500.0, param_desc);
        MV_CC_SetFloatValue(camera_handle_short, "ExposureTime", exposure_time);
        RCLCPP_INFO(this->get_logger(), "Exposure_duan time: %f", exposure_time);
      }

    // Gain
      param_desc.description = "Gain";
      if (camera_handle_long) {
        MV_CC_GetFloatValue(camera_handle_long, "Gain", &f_value);
        param_desc.integer_range[0].from_value = f_value.fMin;
        param_desc.integer_range[0].to_value = f_value.fMax;
      }
      if (camera_handle_short) {
        MV_CC_GetFloatValue(camera_handle_short, "Gain", &f_value);
      }
      double gain = this->declare_parameter("gain", 16.0, param_desc);
      if (camera_handle_long) MV_CC_SetFloatValue(camera_handle_long, "Gain", gain);
      if (camera_handle_short) MV_CC_SetFloatValue(camera_handle_short, "Gain", gain);
      RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // Set image param
    // HiK frame max size: [1440(Width)*1080(Height)] OffsetX:(1440-Width)/2  OffsetY:(1080-Height)/2

      // Width
      int image_width = this->declare_parameter("image_width", 1440);
      if (camera_handle_long) MV_CC_SetIntValue(camera_handle_long, "Width", image_width);
      if (camera_handle_short) MV_CC_SetIntValue(camera_handle_short, "Width", image_width);
      RCLCPP_INFO(this->get_logger(), "Width: %d", image_width);

      // Height
      int image_height = this->declare_parameter("image_height", 1080);
      if (camera_handle_long) MV_CC_SetIntValue(camera_handle_long, "Height", image_height);
      if (camera_handle_short) MV_CC_SetIntValue(camera_handle_short, "Height", image_height);
      RCLCPP_INFO(this->get_logger(), "Height: %d", image_height);

      // OffsetX
      int offset_x = this->declare_parameter("offset_x", 0);
      if (camera_handle_long) MV_CC_SetIntValue(camera_handle_long, "OffsetX", offset_x);
      if (camera_handle_short) MV_CC_SetIntValue(camera_handle_short, "OffsetX", offset_x);

      // OffsetY
      int offset_y = this->declare_parameter("offset_y", 0);
      if (camera_handle_long) MV_CC_SetIntValue(camera_handle_long, "OffsetY", offset_y);
      if (camera_handle_short) MV_CC_SetIntValue(camera_handle_short, "OffsetY", offset_y);
      
      // Set white balance  0:Off 1:Continuous 2:Once
      // 连续白平衡每帧都要跑 AWB 会拖慢帧率，而且会让红/蓝通道增益来回漂，影响敌我颜色判定
      if (camera_handle_long) MV_CC_SetBalanceWhiteAuto(camera_handle_long, 2);
      if (camera_handle_short) MV_CC_SetBalanceWhiteAuto(camera_handle_short, 2);

      // Set FrameRate enable
      if (camera_handle_long) MV_CC_SetBoolValue(camera_handle_long, "AcquisitionFrameRateEnable", true);
      if (camera_handle_short) MV_CC_SetBoolValue(camera_handle_short, "AcquisitionFrameRateEnable", true);

      // Set fps
      float fps = this->declare_parameter("fps", 100.0);
      if (camera_handle_long) MV_CC_SetFloatValue(camera_handle_long, "AcquisitionFrameRate", fps);
      if (camera_handle_short) MV_CC_SetFloatValue(camera_handle_short, "AcquisitionFrameRate", fps);
      // 目标帧率超过相机当前像素格式/带宽能提供的上限时，SetFloatValue 依然返回 MV_OK，
      // 必须读回实际值才能知道有没有被相机悄悄夹住。
      MVCC_FLOATVALUE fps_value{};
      if (camera_handle_long && MV_CC_GetFloatValue(camera_handle_long, "AcquisitionFrameRate", &fps_value) == MV_OK)
        RCLCPP_INFO(this->get_logger(), "FrameRate(long): %.2f (want %.2f)", fps_value.fCurValue, fps);
      if (camera_handle_short && MV_CC_GetFloatValue(camera_handle_short, "AcquisitionFrameRate", &fps_value) == MV_OK)
        RCLCPP_INFO(this->get_logger(), "FrameRate(short): %.2f (want %.2f)", fps_value.fCurValue, fps);

    }
    else if(camera_num==1 && camera_handle_){
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 7500.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
      MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      double gain = this->declare_parameter("gain", 16.0, param_desc);
      MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
      RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

      // Set image param
      // HiK frame max size: [1440(Width)*1080(Height)] OffsetX:(1440-Width)/2  OffsetY:(1080-Height)/2

      // Width
      int image_width = this->declare_parameter("image_width", 1440);
      MV_CC_SetIntValue(camera_handle_, "Width", image_width);
      RCLCPP_INFO(this->get_logger(), "Width: %d", image_width);

      // Height
      int image_height = this->declare_parameter("image_height", 1080);
      MV_CC_SetIntValue(camera_handle_, "Height", image_height);
      RCLCPP_INFO(this->get_logger(), "Height: %d", image_height);

      // OffsetX
      int offset_x = this->declare_parameter("offset_x", 0);
      MV_CC_SetIntValue(camera_handle_, "OffsetX", offset_x);

      // OffsetY
      int offset_y = this->declare_parameter("offset_y", 0);
      MV_CC_SetIntValue(camera_handle_, "OffsetY", offset_y);

      // Set white balance  0:Off 1:Continuous 2:Once
      // 连续白平衡每帧都要跑 AWB 会拖慢帧率，而且会让红/蓝通道增益来回漂，影响敌我颜色判定
      MV_CC_SetBalanceWhiteAuto(camera_handle_, 2);

      // Set FrameRate enable
      MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);

      // Set fps
      float fps = this->declare_parameter("fps", 100.0);
      MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
      // 目标帧率超过相机当前像素格式/带宽能提供的上限时，SetFloatValue 依然返回 MV_OK，
      // 必须读回实际值才能知道有没有被相机悄悄夹住。
      MVCC_FLOATVALUE fps_value{};
      if (MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &fps_value) == MV_OK)
        RCLCPP_INFO(this->get_logger(), "FrameRate: %.2f (want %.2f)", fps_value.fCurValue, fps);
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);  // 保护相机SDK调用
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : parameters)
    {
      if (param.get_name() == "exposure_time")
      {
        int status = 0;
        if(camera_num==2)
          status = MV_CC_SetFloatValue(camera_handle_long, "ExposureTime", param.as_int());
        else
          status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status)
        {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      }
      else if(param.get_name() == "exposure_time1")
      {
        int status = MV_CC_SetFloatValue(camera_handle_short, "ExposureTime", param.as_int());
        if (MV_OK != status)
        {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      }
      else if (param.get_name() == "gain")
      {
        int status = MV_CC_SetFloatValue(camera_handle_long, "Gain", param.as_double());
        status = MV_CC_SetFloatValue(camera_handle_short, "Gain", param.as_double());
        if (MV_OK != status)
        {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      }
      else
      {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  int nRet = MV_OK;
  int camera_num = 0;
  void* camera_handle_ = nullptr;
  void* camera_handle_long = nullptr;
  void* camera_handle_short = nullptr;
  // EnumDevices 返回的指针在下次 Enum 前有效；仅用于构造期 CALLORDER 重开
  MV_CC_DEVICE_INFO * device_info_ = nullptr;
  MV_CC_DEVICE_INFO * device_info_short_ = nullptr;

  std::string camera_info_url;
  std::string frame_id{"camera_optical_frame"};  // 单相机时也要有默认坐标系
  std::string frame_id_short;
  std::string frame_id_long;
  std::string camera_info_url1;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_chang;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_duan;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  sensor_msgs::msg::CameraInfo camera_info_long_msg_;
  sensor_msgs::msg::CameraInfo camera_info_short_msg_;

  // image params

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  rclcpp::Subscription<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;

  std::mutex camera_mutex_;       // 保护相机SDK调用
  jlcv_interfaces::msg::GameStatus latest_game_status_;
  int camera_status_ = 0; // 0-up-short 1-down-long

};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
