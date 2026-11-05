// #include "classifier.hpp"

// #include <yaml-cpp/yaml.h>

// namespace auto_aim
// {
// Classifier::Classifier(const std::string & config_path)
// {
//   auto yaml = YAML::LoadFile(config_path);
//   auto model = yaml["classify_model"].as<std::string>();
//   net_ = cv::dnn::readNetFromONNX(model);
// }

// void Classifier::classify(Armor & armor)
// {
//   cv::Mat gray;
//   cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

//   auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
//   auto x_scale = static_cast<double>(32) / gray.cols;
//   auto y_scale = static_cast<double>(32) / gray.rows;
//   auto scale = std::min(x_scale, y_scale);
//   auto h = static_cast<int>(gray.rows * scale);
//   auto w = static_cast<int>(gray.cols * scale);
//   auto roi = cv::Rect(0, 0, w, h);
//   cv::resize(gray, input(roi), {w, h});

//   auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

//   net_.setInput(blob);
//   cv::Mat outputs = net_.forward();

//   // softmax
//   float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
//   cv::exp(outputs - max, outputs);
//   float sum = cv::sum(outputs)[0];
//   outputs /= sum;

//   double confidence;
//   cv::Point label_point;
//   cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
//   int label_id = label_point.x;

//   armor.confidence = confidence;
//   armor.name = static_cast<ArmorName>(label_id);
// }

// }  // namespace auto_aim
#include "classifier.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
Classifier::Classifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto model = yaml["classify_model"].as<std::string>();
  net_ = cv::dnn::readNetFromONNX(model);
  auto ovmodel = core_.read_model(model);
  compiled_model_ = core_.compile_model(ovmodel, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

void Classifier::classify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }
  // constexpr int lightbar_h = 12;
  // constexpr int small_armor_w = 32;
  // constexpr int big_armor_w = 54;
  // constexpr int warp_h = 28;
  // // 四灯条顶点坐标（透视变换前）
  // std::vector<cv::Point2f> from_points{armor.left.top, armor.right.top, armor.right.bottom, armor.left.bottom};
  // std::cout << from_points << std::endl;

  // // 四灯条顶点坐标（透视变换后）
  // const int lightbar_top_y = (warp_h - lightbar_h) / 2 - 1;
  // const int lightbar_bottom_y = lightbar_top_y + lightbar_h;
  // const int warp_w = (armor.type == auto_aim::ArmorType::big) ? big_armor_w : small_armor_w;
  // std::vector<cv::Point2f> to_points{
  //   cv::Point2f(0, lightbar_top_y), cv::Point2f(warp_w - 1, lightbar_top_y), cv::Point2f(warp_w - 1, lightbar_bottom_y),
  //   cv::Point2f(0, lightbar_bottom_y)};
  // std::cout << to_points << std::endl;
  // std::cout << "-----------------------------------" << std::endl;

  // // 进行透视变换
  // auto transform = cv::getPerspectiveTransform(from_points, to_points);
  // cv::warpPerspective(gray, gray, transform, cv::Size(warp_w, warp_h));

  // cv::imshow("gray Image", gray);
  // cv::waitKey(1);  // 等待用户按键
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});

  // cv::imshow("Input Image", input);
  // cv::waitKey(1);  // 等待用户按键
  auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

  net_.setInput(blob);
  cv::Mat outputs = net_.forward();

  // softmax
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

void Classifier::ovclassify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  // Resize image to 32x32
  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});
  // Normalize the input image to [0, 1] range
  input.convertTo(input, CV_32F, 1.0 / 255.0);

  ov::Tensor input_tensor(ov::element::f32, {1, 1, 32, 32}, input.data);

  ov::InferRequest infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat outputs(1, 9, CV_32F, output_tensor.data());

  // Softmax
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

}  // namespace auto_aim