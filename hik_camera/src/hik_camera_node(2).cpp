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

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:

      explicit HikCameraNode(const rclcpp::NodeOptions& options) : Node("camera_node", options)
    {
      RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");
      
      MV_CC_DEVICE_INFO_LIST device_list;
      // enum device
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      frame_id_short = "camera_optical_frame1";
      frame_id_long = "camera_optical_frame";
      frame_id = frame_id_long;
      RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

      while (device_list.nDeviceNum == 0 && rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      }

      camera_num = device_list.nDeviceNum;
      // const char* serial_chang = this->declare_parameter("camera_serial", "DA183077").c_str();
      // const char* serial_duan = this->declare_parameter("camera_serial1", "DA071003").c_str();
      const char* serial_chang = this->declare_parameter("camera_serial", "DA0709996").c_str();
      const char* serial_duan = this->declare_parameter("camera_serial1", "DA1830770").c_str();      
      
      std::cout<<"serial_chang "<<serial_chang<<"\nserial_duan "<<serial_duan<<std::endl;
      if(camera_num==2)
      {
        auto serial0 = device_list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chSerialNumber;
        auto serial1 = device_list.pDeviceInfo[1]->SpecialInfo.stUsb3VInfo.chSerialNumber;

        std::cout<<"serial0 "<<serial0<<"\nserial1 "<<serial1<<std::endl;
        std::cout<<strcmp(reinterpret_cast<char*>(serial0),serial_duan)<<std::endl;
        if(strcmp(reinterpret_cast<char*>(serial0),serial_duan)==0)
        {
          MV_CC_CreateHandle(&camera_handle_long, device_list.pDeviceInfo[1]);
          MV_CC_CreateHandle(&camera_handle_short, device_list.pDeviceInfo[0]);
        }
        else{
          MV_CC_CreateHandle(&camera_handle_long, device_list.pDeviceInfo[0]);
          MV_CC_CreateHandle(&camera_handle_short, device_list.pDeviceInfo[1]);
        }
        MV_CC_OpenDevice(camera_handle_long);
        MV_CC_OpenDevice(camera_handle_short);
      }
      else
      {
        MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
        MV_CC_OpenDevice(camera_handle_);
      }
      

      MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
      std::cout<<"camera_num"<<camera_num<<std::endl;
      
      declareParameters();
      // Get camera infomation 
      MV_CC_GetImageInfo(camera_handle_long, &img_info_);
      image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
      MV_CC_StartGrabbing(camera_handle_long);
      convert_param_chang.nWidth = img_info_.nWidthValue;
      convert_param_chang.nHeight = img_info_.nHeightValue;
      convert_param_chang.enDstPixelType = PixelType_Gvsp_RGB8_Packed;  // PixelType_Gvsp_RGB8_Packed
      
      MV_CC_GetImageInfo(camera_handle_short, &img_info_);
      image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
      MV_CC_StartGrabbing(camera_handle_short);
      convert_param_duan.nWidth = img_info_.nWidthValue;
      convert_param_duan.nHeight = img_info_.nHeightValue;
      convert_param_duan.enDstPixelType = PixelType_Gvsp_RGB8_Packed;  // PixelType_Gvsp_RGB8_Packed

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
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      }

      params_callback_handle_ = this->add_on_set_parameters_callback(
          std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

      camera_handle_ = camera_handle_long;
      convert_param_.nWidth = convert_param_chang.nWidth;
      convert_param_.nHeight = convert_param_chang.nHeight;
      game_status_sub_ = this->create_subscription<jlcv_interfaces::msg::GameStatus>(//收节点信息，获取GameStatus
      "/game_status", rclcpp::SensorDataQoS(), [this](jlcv_interfaces::msg::GameStatus::ConstSharedPtr game_status) {

        if(camera_status != game_status->camera_status)
        {
          // std::cout<<"camera_status:"<<camera_status<<"\n";
          camera_status = game_status->camera_status;
          camera_status = 1;
          if(camera_status == 0){
            frame_id = frame_id_long;
            camera_handle_ = camera_handle_long;
            convert_param_.nWidth = convert_param_chang.nWidth;
            convert_param_.nHeight = convert_param_chang.nHeight;
            if (camera_info_manager_->validateURL(camera_info_url))
            {
              camera_info_manager_->loadCameraInfo(camera_info_url);
              camera_info_msg_ = camera_info_manager_->getCameraInfo();
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
            }
          }else{
            frame_id = frame_id_short;
            camera_handle_ = camera_handle_short;
            convert_param_.nWidth = convert_param_duan.nWidth;
            convert_param_.nHeight = convert_param_duan.nHeight;
            if (camera_info_manager_->validateURL(camera_info_url1))
            {
              // camera_info_manager_->loadCameraInfo(camera_info_url1);
              camera_info_msg_ = camera_info_manager_->getCameraInfo();
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url1.c_str());
            }
          }
          // std::cout << "final_status:" << camera_status << "\n";
        }
      });  
      capture_thread_ = std::thread{ [this]() -> void {
        MV_FRAME_OUT out_frame;

        RCLCPP_INFO(this->get_logger(), "Publishing image!");
        image_msg_.encoding = "rgb8";

        while (rclcpp::ok())
        {
          nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
          if (MV_OK == nRet)
          {
            image_msg_.header.frame_id = frame_id;
            convert_param_.pDstBuffer = image_msg_.data.data();
            convert_param_.nDstBufferSize = image_msg_.data.size();
            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

            image_msg_.header.stamp = this->now();

            MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
            MV_CC_SetBayerCvtQuality(camera_handle_, 1);

            image_msg_.height = out_frame.stFrameInfo.nHeight;
            image_msg_.width = out_frame.stFrameInfo.nWidth;
            image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
            image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

            camera_info_msg_.header = image_msg_.header;
            // 替换第 217-220 行
            auto unique_img_msg = std::make_unique<sensor_msgs::msg::Image>();
            unique_img_msg->header = image_msg_.header;
            unique_img_msg->height = image_msg_.height;
            unique_img_msg->width = image_msg_.width;
            unique_img_msg->encoding = image_msg_.encoding;
            unique_img_msg->step = image_msg_.step;
            unique_img_msg->data.assign(image_msg_.data.begin(), image_msg_.data.end()); // 深拷贝

            image_pub_->publish(std::move(unique_img_msg));

            camera_info_pub_->publish(camera_info_msg_);

            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            fail_conut_ = 0;
          }
          else
          { 
            RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_StartGrabbing(camera_handle_);
            fail_conut_++;
          }

          // 连续失败次数超过5次，输出相机失败的错误信息并关闭节点
          if (fail_conut_ > 5)
          {
            RCLCPP_FATAL(this->get_logger(), "Camera failed!");
            rclcpp::shutdown();
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

    // 检查相机句柄是否存在，如果存在则执行相机资源的清理操作
    if (camera_handle_)
    {
      // 停止相机的图像抓取操作
      MV_CC_StopGrabbing(camera_handle_);
      // 关闭相机设备
      MV_CC_CloseDevice(camera_handle_);
      // 销毁相机句柄
      MV_CC_DestroyHandle(&camera_handle_);
    }

    // 输出日志信息，表明 HikCameraNode 已被销毁
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:

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
    MV_CC_GetFloatValue(camera_handle_long, "ExposureTime", &f_value);
    MV_CC_GetFloatValue(camera_handle_short, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 7500.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_long, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure_chang time: %f", exposure_time);

    exposure_time = this->declare_parameter("exposure_time1", 7500.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_short, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure_duan time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_long, "Gain", &f_value);
    MV_CC_GetFloatValue(camera_handle_short, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", 16.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_long, "Gain", gain);
    MV_CC_SetFloatValue(camera_handle_short, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // Set image param
    // HiK frame max size: [1440(Width)*1080(Height)] OffsetX:(1440-Width)/2  OffsetY:(1080-Height)/2

    // Width
    int image_width = this->declare_parameter("image_width", 1440);
    MV_CC_SetIntValue(camera_handle_long, "Width", image_width);
    MV_CC_SetIntValue(camera_handle_short, "Width", image_width);
    RCLCPP_INFO(this->get_logger(), "Width: %d", image_width);

    // Height
    int image_height = this->declare_parameter("image_height", 1080);
    MV_CC_SetIntValue(camera_handle_long, "Height", image_height);
    MV_CC_SetIntValue(camera_handle_short, "Height", image_height);
    RCLCPP_INFO(this->get_logger(), "Height: %d", image_height);

    // OffsetX
    int offset_x = this->declare_parameter("offset_x", 0);
    MV_CC_SetIntValue(camera_handle_long, "OffsetX", offset_x);
    MV_CC_SetIntValue(camera_handle_short, "OffsetX", offset_x);

    // OffsetY
    int offset_y = this->declare_parameter("offset_y", 0);
    MV_CC_SetIntValue(camera_handle_long, "OffsetY", offset_y);
    MV_CC_SetIntValue(camera_handle_short, "OffsetY", offset_y);
    // Set white balance  0:Off 1:Continuous 2:Once
    MV_CC_SetBalanceWhiteAuto(camera_handle_long, 1);
    MV_CC_SetBalanceWhiteAuto(camera_handle_short, 1);

    // Set FrameRate enable
    MV_CC_SetBoolValue(camera_handle_long, "AcquisitionFrameRateEnable", true);
    MV_CC_SetBoolValue(camera_handle_short, "AcquisitionFrameRateEnable", true);

    // Set fps
    float fps = this->declare_parameter("fps", 100.0);
    MV_CC_SetFloatValue(camera_handle_long, "AcquisitionFrameRate", fps);
    MV_CC_SetFloatValue(camera_handle_short, "AcquisitionFrameRate", fps);

    }
    else{
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
    MV_CC_SetBalanceWhiteAuto(camera_handle_, 1);

    // Set FrameRate enable
    MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);

    // Set fps
    float fps = this->declare_parameter("fps", 100.0);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
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

  sensor_msgs::msg::Image image_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  int nRet = MV_OK;
  int camera_num = 0;
  void* camera_handle_;
  void* camera_handle_long;
  void* camera_handle_short;

  std::string camera_info_url;
  std::string frame_id;
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

  // image params

  int fail_conut_ = 0;
  std::thread capture_thread_;


  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  int camera_status = -2;
  rclcpp::Subscription<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;

};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
