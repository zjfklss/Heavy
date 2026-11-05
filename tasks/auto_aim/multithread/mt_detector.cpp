#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <cstring>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug) : yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  use_roi_ = yaml["use_roi"].as<bool>();
  std::cout << "use roi: " << use_roi_ << std::endl;
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  std::cout << "ROI: x=" << x << ", y=" << y << ", width=" << width << ", height=" << height << std::endl;
  roi_ = cv::Rect(x, y, width, height);

  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

  tools::logger()->info("[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat raw_img, std::chrono::steady_clock::time_point t)
{
  cv::Mat img;
  if (use_roi_) {
    img = raw_img(roi_);
  } else {
    img = raw_img;
  }

  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  auto input_port = compiled_model_.input();
  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3});
  std::memcpy(input_tensor.data(), input.data, input.total() * input.elemSize());

  infer_request.set_input_tensor(input_tensor);
  infer_request.start_async();
  queue_.push({raw_img.clone(), t, std::move(infer_request)});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [raw_img, t, infer_request] = queue_.pop();
  cv::Mat img;
  if (use_roi_) {
    img = raw_img(roi_);
  } else {
    img = raw_img;
  }
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, raw_img, 0);

  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::debug_pop()
{
  return process(queue_.pop());
}

std::optional<std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>>
MultiThreadDetector::debug_pop_for(std::chrono::milliseconds timeout)
{
  Frame frame;
  if (!queue_.try_pop_for(frame, timeout)) return std::nullopt;
  return process(std::move(frame));
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::process(Frame frame)
{
  auto [raw_img, t, infer_request] = std::move(frame);
  cv::Mat img;
  if (use_roi_) {
    img = raw_img(roi_);
  } else {
    img = raw_img;
  }
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, raw_img, 0);

  return {raw_img, std::move(armors), t};
}

}  // namespace multithread

}  // namespace auto_aim
