// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file armor.hpp
 * @brief 装甲检测中使用的光条与装甲数据结构。
 *
 * 定义了检测到的光条 `Light` 和由光条配对出的装甲板 `Armor` 的数据容器。
 * 它们保存图像几何信息（中心、角点）、几何评估量（长度、宽度、倾斜角）
 * 以及识别输出（数字标签、置信度）。
 */

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

#include <algorithm>
#include <string>

namespace rm_auto_aim
{

// ======================== 枚举与常量 ========================

/// 装甲名称枚举
enum ArmorName
{
  one, two, three, four, five, sentry, outpost, base, not_armor
};

/// 装甲名称字符串映射
const std::vector<std::string> ARMOR_NAMES = {
  "sentry", "one", "two", "three", "four", "five", "outpost", "base", "not_armor"
};

/// 装甲板颜色枚举
enum Color { red, blue, extinguish, purple };

/// 装甲板颜色字符串映射
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

/// 赛场的两种颜色ID
const int RED  = 0;
const int BLUE = 1;

/// 装甲类型枚举
enum class ArmorType { SMALL, LARGE, INVALID };

/// 装甲类型字符串映射
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

// ======================== Light ========================

/**
 * @brief 代表一个检测到的光条。
 *
 * 继承自 cv::RotatedRect，计算上端/下端、长度、宽度和倾斜角。
 */
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

// ======================== Armor ========================

/**
 * @brief 代表一个配对的装甲板，由两个 Light 对象构成。
 *
 * 存储左右灯条、中心点、装甲板类型和数字识别结果。
 */
struct Armor
{
  Armor() = default;

  /**
   * @brief 从两个 Light 对象构造 Armor 对象
   * @deprecated 该构造函数已被基于 YOLO 输出的构造函数取代。
   */
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  /**
   * @brief 根据 YOLO 的输出构造 Armor 对象
   * @param color 颜色ID
   * @param num 数字ID
   * @param conf 置信度
   * @param bbox 装甲边界框
   * @param keypoints 装甲关键点（4个角点）
   */
  Armor(
    int color, int num, float & conf, const cv::Rect & bbox,
    std::vector<cv::Point2f> keypoints)
  : confidence(conf), box(bbox), color_id(color), num_id(num), armor_keypoints(keypoints)
  {
    center = (armor_keypoints[0] + armor_keypoints[1] +
              armor_keypoints[2] + armor_keypoints[3]) / 4;

    left_light.top     = armor_keypoints[0];
    right_light.top    = armor_keypoints[1];
    right_light.bottom = armor_keypoints[2];
    left_light.bottom  = armor_keypoints[3];

    left_light.tilt_angle = std::atan2(
      std::abs(left_light.top.x - left_light.bottom.x),
      std::abs(left_light.top.y - left_light.bottom.y));
    right_light.tilt_angle = std::atan2(
      std::abs(right_light.top.x - right_light.bottom.x),
      std::abs(right_light.top.y - right_light.bottom.y));

    number = ARMOR_NAMES[num];
    left_light.color  = color % 2;
    right_light.color = color % 2;

    classfication_result = ARMOR_NAMES[num] + "-" + COLORS[color] + ":" + std::to_string(confidence);

    if (num_id == 1) {
      type = ArmorType::LARGE;
    } else if (num_id == 8) {
      type = ArmorType::INVALID;
    } else {
      type = ArmorType::SMALL;
    }
  }

  // ---------- 成员变量 ----------
  Light left_light, right_light;            ///< 左右灯条
  cv::Point2f center;                       ///< 装甲板中心点
  ArmorType type;                           ///< 装甲板类型
  std::string number;                       ///< 装甲板数字符号
  float confidence;                         ///< YOLO 检测置信度
  std::string classfication_result;         ///< 识别结果字符串
  cv::Rect box;                             ///< 边界框
  int color_id;                             ///< 颜色ID
  int num_id;                               ///< 数字ID
  std::vector<cv::Point2f> armor_keypoints; ///< 关键点（角点）

  float reprojection_error = 1e9f;          ///< PnP 重投影误差
  float light_mean_length  = 0.0f;          ///< 灯条平均长度
  bool  refined            = false;         ///< 是否经过传统视觉优化

  cv::Mat number_img;
  uint8_t label_id;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
