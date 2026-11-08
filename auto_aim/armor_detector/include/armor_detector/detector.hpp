// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file detector.hpp
 * @brief 用于提取光条并配对为装甲的检测流水线。
 *
 * `Detector` 类封装图像预处理、光条候选提取、光条配对为装甲以及调试信息的采集。
 * 它还持有一个 `NumberClassifier` 实例用于从装甲板中提取并识别数字。
 *
 * 接口契约（输入/输出）：
 *  - 输入：RGB 图像（cv::Mat），通过 `detect()` 提供
 *  - 输出：包含检测到的装甲候选（vector<Armor>），每个候选含数字裁剪图像与分类结果
 */

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
// #include "armor_detector/number_classifier.hpp"
#include "jlcv_interfaces/msg/debug_armors.hpp"
#include "jlcv_interfaces/msg/debug_lights.hpp"

#include <openvino/openvino.hpp>
namespace rm_auto_aim
{
/**
 * @brief 主要检测流水线类。
 *
 * 主要功能：
 *  - 将传入的RGB图像预处理成二值掩码以突出目标颜色区域；
 *  - 通过YOLO检测识别装甲板
 *  - 从检测到的装甲板区域提取光条候选并根据灯条得分进行角点优化；
 *  - 将光条配对成装甲假设并进行几何校验；
 *  - 提供调试信息以便可视化中间结果。
 */
class Detector
{
public:

  /**
   * @brief 用于过滤候选灯条的参数结构体。
   */
  struct LightParams
  {
    /// 灯条 长边/短边 的最小比例
    double min_ratio = 1.5;
    /// 灯条 长边/短边 的最大比例
    double max_ratio = 12;
    /// 灯条最大倾斜角度（度）
    double max_angle;
  };

  /**
   * @brief 用于判断装甲板合理性的参数结构体。
   */
  struct ArmorParams
  {
    /// 灯条配对时的最小长度比
    double min_light_ratio;
    /// @brief 小装甲板的距离范围（）最大
    double min_small_center_distance;
    /// @brief 小装甲板的距离范围（最大）
    double max_small_center_distance;
    /// @brief 大装甲
    double min_large_center_distance;
    /// @brief 大装甲板的距离范围（最大）
    double max_large_center_distance;
    /// @brief 两灯条之间的最大角度
    double max_angle;
  };

  /**
   * @brief 装甲板检测器的构造函数
   * @param bin_thres 预处理的二值化与之 (unused in current
   *        algorithm, kept for compatibility)
   * @param color 敌方的装甲板灯条颜色
   * @param l 灯条尺寸参数
   * @param a 装甲板尺寸参数
   * @deprecated 这个传统构造函数已被配置文件构造函数（YOLO）取代，建议使用后者。
   */
  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  
  /**
   * @brief 装甲板yolo检测器的构造函数
   * @param config_path 配置文件路径
   * 
   * 该构造函数从指定的YAML配置文件中读取参数，并初始化YOLO检测器。
   * 关键初始化内容：enemy_color, 模型路径等。
   */
  Detector(const std::string &config_path);


  /**
   * @brief 利用传统视觉对YOLO检测到的装甲板(ROI)进行精修
   * @param armors YOLO检测到的装甲板列表
   * @param raw_img 原始完整图像
   * 
   * 该方法在每个装甲板的ROI区域内提取光条候选，并根据灯条得分进行角点优化，从而提升装甲板定位的准确性。
   * 首先会根据装甲板的颜色与敌方颜色进行初步筛选，然后在ROI内查找灯条并计算配对得分，选择最佳灯条对进行优化。
   * 该方法会修改传入的装甲板列表，更新每个装甲板的关键点位置。
   * 调用yoloFindLights()函数对ROI内灯条进行提取。调用getArmorCost()函数计算灯条配对得分。
   */
  void refineArmors(std::vector<Armor> &armors, const cv::Mat & raw_img);

  /**
   * @brief 调用YOLO检测装甲板
   * @param raw_img 输入的原始RGB图像
   * @return std::vector<Armor> 检测到的装甲板列表
   * 
   * 该方法使用YOLO模型在传入的图像中检测装甲板，并返回检测到的装甲板候选列表。
   */
  std::vector<Armor> yolodetect(cv::Mat &raw_img);

  /**
   * @brief 从yolo检测出的候选区域里提取出符合条件的灯条
   * @param roi_img_bgr 原始RGB图像 (用于颜色检查)
   * @return std::vector<Light> 验证通过的灯条列表
   * 
   * 该方法在传入的ROI图像中查找轮廓，拟合旋转矩形，并根据预设的灯条参数筛选出合理的灯条候选。
   * 返回的灯条按x坐标排序，便于后续的配对处理。
   */
  std::vector<Light> yoloFindLights(const cv::Mat & roi_img_bgr, Armor &armor);

  /**
   * @brief 绘制检测框
   * @param img 待绘制的图像
   * @param armors 装甲板列表
   * 
   * 在传入的图像上绘制每个装甲板的边界框和相关信息，便于可视化检测结果。
   */
  void draw_detections(cv::Mat & img, const std::vector<Armor> & armors);

  // 二值化阈值
  int binary_thres;

  // 敌方颜色
  int enemy_color;

  // 灯条参数
  LightParams l;

  // 装甲板参数
  ArmorParams a;

  // 二值化图像
  cv::Mat binary_img;

  // 灯条调试消息
  jlcv_interfaces::msg::DebugLights debug_lights;

  // 装甲板调试消息
  jlcv_interfaces::msg::DebugArmors debug_armors;


private:

  /// @brief 模型路径
  std::string model_path_;

  /// @brief 推理所用设备
  std::string device_;

  /// @brief 调试图片保存路径
  std::string save_path_, debug_path_;

  /// @brief 模型中类别个数
  const int class_num_ = 13;

  /// @brief nms阈值
  const float nms_threshold_ = 0.3;
  
  /// @brief 置信度阈值
  const float score_threshold_ = 0.7;
  
  /// @brief 最小置信度
  double min_confidence_;

  /// @brief 二值化阈值
  double binary_threshold_;

  /// @brief OpenVIno核心对象
  ov::Core core_;

  /// @brief 编译后的对象
  ov::CompiledModel compiled_model_;

  /// @brief roi区域
  cv::Rect roi_;

  /// @brief 偏移量
  cv::Point2f offset_;


  /**
   * @brief 计算相对于图像中心归一化的坐标
   * @param bgr_img 输入的BGR图像
   * @param center 装甲板中心点坐标
  */
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  /**
   * @brief 解析YOLO模型的输出，生成装甲板列表
   * @param scale 图像缩放比例
   * @param output 模型输出的张量
   * @param bgr_img 输入的BGR图像
   * @return std::vector<Armor> 解析得到的装甲板列表
   */
  std::vector<Armor> postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img);

  /**
   * @brief 解析YOLO模型的输出，生成装甲板列表
   * @param scale 图像缩放比例
   * @param output 模型输出的张量
   * @param bgr_img 输入的BGR图像
   * @return std::vector<Armor> 解析得到的装甲板列表
   */
  std::vector<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img);

  /**
   * @brief 计算一对灯条配对成装甲的惩罚分数
   * @param l1 第一个灯条
   * @param l2 第二个灯条
   * @param target_type 目标类型（大装甲或小装甲）
   * @param roi_center ROI区域中心点坐标
   * @return double 配对的惩罚分数，分数越低表示配对越合理
   * 
   * 该函数根据灯条的长度比、中心距离、角度等多项指标计算配对的惩罚分数。
   * 不同类型的装甲（大装甲或小装甲）使用不同的距离范围进行评估。
   * 可根据实际情况调整各项指标的权重以优化配对效果。
   */
  double getArmorCost(const Light & l1, const Light & l2, int target_type, const cv::Point2f & roi_center);
  

  /**
   * @brief sigmoid 函数用于yolo后处理
   * @param x 输入的原始分数
   * @return double sigmoid运算后的分数
   * 
   * 常见 sigmoid函数
   */
  double sigmoid(double x);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
