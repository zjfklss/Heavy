// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

/**
 * @file detector.cpp
 * @brief 实现用于查找光条并将其配对成装甲候选的图像处理流水线。
 *
 * 主要步骤：
 *  - `yolodetect`：通过 YOLO 检测识别装甲板；
 *  - `refineArmors`：从检测到的装甲板区域提取光条候选并根据灯条得分进行角点优化；
 *  - `yoloFindLights`：在 ROI 图像中查找轮廓，拟合旋转矩形，并根据预设参数筛选灯条候选；
 *  - `getArmorCost`：计算装甲候选的合理性得分，用于后续筛选；
 *  - `draw_detections`：在图像上绘制检测到的装甲板边界框和相关信息。
 */

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>

#include "armor_detector/detector.hpp"
#include "jlcv_interfaces/msg/debug_armor.hpp"
#include "jlcv_interfaces/msg/debug_light.hpp"

#include <yaml-cpp/yaml.h>
#include <openvino/openvino.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rm_auto_aim
{

Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), enemy_color(color), l(l), a(a)
{
}

Detector::Detector(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");

  model_path_ = pkg_path + yaml["yolov5_model_path"].as<std::string>();
  enemy_color = yaml["enemy_color"].as<int>();

  l.min_ratio = yaml["min_lightbar_ratio"].as<double>();
  l.max_ratio = yaml["max_lightbar_ratio"].as<double>();
  l.max_angle = yaml["max_rectangular_error"].as<double>();

  device_           = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_   = yaml["min_confidence"].as<double>();

  int x      = yaml["roi"]["x"].as<int>();
  int y      = yaml["roi"]["y"].as<int>();
  int width  = yaml["roi"]["width"].as<int>();
  int height = yaml["roi"]["height"].as<int>();
  roi_    = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  // 加载 OpenVINO 模型
  auto model = core_.read_model(model_path_);

  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::vector<Armor> Detector::yolodetect(cv::Mat & raw_img)
{
  if (raw_img.empty()) {
    std::cerr << "[Detector] Empty image, camera drop!" << std::endl;
    return std::vector<Armor>();
  }

  cv::Mat bgr_img = raw_img;

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // 预处理：缩放并填充到 640x640
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // 推理
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // 后处理
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img);
}

std::vector<Armor> Detector::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img)
{
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    double score = sigmoid(output.at<float>(r, 8));
    if (score < score_threshold_) continue;

    // 解析颜色和类别独热向量
    cv::Mat color_scores   = output.row(r).colRange(9, 13);
    cv::Mat classes_scores = output.row(r).colRange(13, 22);
    cv::Point class_id, color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);

    // 提取4个关键点并排序为 TL, TR, BR, BL 顺序
    std::vector<cv::Point2f> raw_points;
    for (int k = 0; k < 4; k++) {
      float x = output.at<float>(r, k * 2) / scale;
      float y = output.at<float>(r, k * 2 + 1) / scale;
      raw_points.push_back(cv::Point2f(x, y));
    }

    // 按 x 坐标排序，再分别对左、右两列按 y 排序
    std::sort(raw_points.begin(), raw_points.end(),
      [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

    if (raw_points[0].y > raw_points[1].y) std::swap(raw_points[0], raw_points[1]);
    if (raw_points[2].y > raw_points[3].y) std::swap(raw_points[2], raw_points[3]);

    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.push_back(raw_points[0]); // TL
    armor_key_points.push_back(raw_points[2]); // TR
    armor_key_points.push_back(raw_points[3]); // BR
    armor_key_points.push_back(raw_points[1]); // BL

    // 计算 bounding box
    float min_x = armor_key_points[0].x, max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y, max_y = armor_key_points[0].y;
    for (size_t i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    color_ids.emplace_back(color_id.x);
    num_ids.emplace_back(class_id.x);
    boxes.emplace_back(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::vector<Armor> armors;
  for (const auto & i : indices) {
    armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
  }

  return armors;
}

void Detector::refineArmors(std::vector<Armor> & armors, const cv::Mat & raw_img)
{
  for (auto & armor : armors) {
    // 跳过非敌方颜色的装甲
    if (COLORS[armor.color_id] != (enemy_color == RED ? "red" : "blue")) {
      continue;
    }

    // 1. 确定并扩展 ROI 区域，防止灯条被裁断
    cv::Rect roi = armor.box;
    int padding_w = roi.width  * 0.2;
    int padding_h = roi.height * 0.3;
    roi.x -= padding_w;
    roi.y -= padding_h;
    roi.width  += 2 * padding_w;
    roi.height += 2 * padding_h;

    // 2. 安全边界检查
    roi &= cv::Rect(0, 0, raw_img.cols, raw_img.rows);
    if (roi.empty()) continue;

    // 3. 截取 ROI 并查找灯条
    cv::Mat roi_img_bgr = raw_img(roi);
    std::vector<Light> roi_lights = yoloFindLights(roi_img_bgr, armor);

    if (roi_lights.size() < 2) {
      continue;
    }

    // 4. 基于惩罚函数的灯条配对
    double min_cost = 1e9;
    int best_i = -1, best_j = -1;
    cv::Point2f roi_center(roi.width / 2.0f, roi.height / 2.0f);

    // 1号(英雄) / 7号(基地) 为大装甲，其余为小装甲
    int target_type = (armor.num_id == 1) ? 1 : 0;

    for (size_t i = 0; i < roi_lights.size(); i++) {
      for (size_t j = i + 1; j < roi_lights.size(); j++) {
        double cost = getArmorCost(roi_lights[i], roi_lights[j], target_type, roi_center);
        if (cost < min_cost) {
          min_cost = cost;
          best_i = i;
          best_j = j;
        }
      }
    }

    if (min_cost > 5.0 || best_i == -1) {
      armor.refined = false;
      armor.light_mean_length = 0.0f;
      continue;
    }

    // 5. 将最佳灯条对的坐标从 ROI 坐标转换为原图坐标
    Light l1 = roi_lights[best_i];
    Light l2 = roi_lights[best_j];

    auto offsetPoint = [&](cv::Point2f & p) { p.x += roi.x; p.y += roi.y; };
    offsetPoint(l1.top); offsetPoint(l1.bottom); offsetPoint(l1.center);
    offsetPoint(l2.top); offsetPoint(l2.bottom); offsetPoint(l2.center);

    if (l1.center.x < l2.center.x) {
      armor.left_light = l1; armor.right_light = l2;
    } else {
      armor.left_light = l2; armor.right_light = l1;
    }

    armor.center = (armor.left_light.center + armor.right_light.center) / 2;

    // 更新关键点：TL, TR, BR, BL（与 PnP 模型点顺序对应）
    armor.armor_keypoints.clear();
    armor.armor_keypoints.push_back(armor.left_light.top);     // TL
    armor.armor_keypoints.push_back(armor.right_light.top);    // TR
    armor.armor_keypoints.push_back(armor.right_light.bottom); // BR
    armor.armor_keypoints.push_back(armor.left_light.bottom);  // BL

    armor.refined = true;
    armor.light_mean_length = (armor.left_light.length + armor.right_light.length) / 2.0f;
  }
}

std::vector<Light> Detector::yoloFindLights(const cv::Mat & roi_img_bgr, Armor & armor)
{
  std::vector<cv::Mat> channels;
  cv::split(roi_img_bgr, channels);

  cv::Mat gray_img;
  cv::cvtColor(roi_img_bgr, gray_img, cv::COLOR_BGR2GRAY);

  // 颜色差分：红色取 R-B，蓝色取 B-R
  cv::Mat color_diff;
  if (enemy_color == 0) {
    cv::subtract(channels[2], channels[0], color_diff);
  } else {
    cv::subtract(channels[0], channels[2], color_diff);
  }

  // 颜色阈值
  cv::Mat binary_color;
  cv::threshold(color_diff, binary_color, 60, 255, cv::THRESH_BINARY);

  // 亮度阈值（解决灯条中心过曝变白问题）
  cv::Mat binary_brightness;
  cv::threshold(gray_img, binary_brightness, 200, 255, cv::THRESH_BINARY);

  // 融合颜色与亮度（OR 取并集，保留过曝中心）
  cv::Mat final_binary;
  cv::bitwise_or(binary_color, binary_brightness, final_binary);

  // 形态学膨胀，连接断开的区域
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(final_binary, final_binary, kernel);

  // 查找轮廓并筛选灯条
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(final_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<Light> lights;
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    cv::RotatedRect r_rect = cv::minAreaRect(contour);

    // 角度筛选（灯条近似竖直）
    float angle = r_rect.angle;
    if (r_rect.size.width > r_rect.size.height) angle -= 90;
    if (std::abs(angle) > 35.0) continue;

    // 长宽比筛选（灯条是细长的）
    float w = r_rect.size.width;
    float h = r_rect.size.height;
    if (h < w) std::swap(h, w);
    float ratio = h / w;
    if (ratio < 1.5 || ratio > 12.0) continue;

    // 面积筛选
    if (r_rect.size.area() < 10 || r_rect.size.area() > roi_img_bgr.total() * 0.6) continue;

    Light light(r_rect);
    light.color = enemy_color;
    lights.emplace_back(light);
  }

  // 按 x 坐标排序，方便后续配对
  std::sort(lights.begin(), lights.end(),
    [](const Light & a, const Light & b) { return a.center.x < b.center.x; });

  return lights;
}

double Detector::getArmorCost(
  const Light & l1, const Light & l2, int target_type, const cv::Point2f & roi_center)
{
  // 颜色检查
  if (l1.color != enemy_color || l2.color != enemy_color) {
    return 1e9;
  }

  // 角度差（平行度）
  float angle_diff = std::abs(l1.tilt_angle - l2.tilt_angle);
  double cost_angle = angle_diff / 10.0;

  // 长度差（高度相似性）
  double avg_len = (l1.length + l2.length) / 2.0;
  double len_diff = std::abs(l1.length - l2.length);
  double cost_len = len_diff / (avg_len * 0.5);

  // 距离中心偏差
  cv::Point2f pair_center = (l1.center + l2.center) / 2.0;
  double dist_to_center = cv::norm(pair_center - roi_center);
  double cost_center = dist_to_center / (roi_center.x * 0.4);

  // 长宽比
  double dis = cv::norm(l1.center - l2.center);
  double ratio = dis / avg_len;
  double target_ratio = (target_type == 1) ? 4.182 : 2.436;  // 大装甲 : 小装甲
  double ratio_diff = std::abs(ratio - target_ratio);
  double cost_ratio = ratio_diff / 0.5;

  // Y轴错位
  double y_diff = std::abs(l1.center.y - l2.center.y);
  double cost_y = y_diff / (avg_len * 0.5);

  // 加权总分
  constexpr double w_angle  = 1.5;
  constexpr double w_len    = 1.0;
  constexpr double w_center = 2.5;
  constexpr double w_ratio  = 3.0;
  constexpr double w_y      = 1.5;

  return w_angle  * cost_angle  +
         w_len    * cost_len    +
         w_center * cost_center +
         w_ratio  * cost_ratio  +
         w_y      * cost_y;
}

cv::Point2f Detector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void Detector::draw_detections(cv::Mat & img, const std::vector<Armor> & armors)
{
  const std::vector<cv::Scalar> colors = {
    cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255),
    cv::Scalar(0, 255, 0), cv::Scalar(255, 255, 0)
  };

  for (const auto & armor : armors) {
    cv::rectangle(img, armor.box, colors[armor.color_id], 5);
    cv::putText(img, armor.classfication_result,
                cv::Point(armor.box.tl().x, armor.box.tl().y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 2.0, colors[armor.color_id], 2);
  }
}

double Detector::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::vector<Armor> Detector::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img)
{
  return parse(scale, output, bgr_img);
}

}  // namespace rm_auto_aim
