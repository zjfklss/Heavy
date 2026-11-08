// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR_TRADITIONAL__DETECTOR_HPP_
#define ARMOR_DETECTOR_TRADITIONAL__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "jlcv_interfaces/msg/debug_armors.hpp"
#include "jlcv_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
class TraditionalDetector
{
public:

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

  TraditionalDetector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  int binary_thres;
  int enemy_color;
  LightParams l;
  ArmorParams a;
  cv::Mat binary_img;
  std::unique_ptr<NumberClassifier> classifier;

  jlcv_interfaces::msg::DebugLights debug_lights;
  jlcv_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light & possible_light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
