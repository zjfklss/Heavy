#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <optional>

#include "buff_type.hpp"
#include "tools/img_tools.hpp"
#include "yolo11_buff.hpp"
const int LOSE_MAX = 20;  // 丢失的阙值
namespace auto_buff
{
// 能量机关检测：YOLO11 关键点模型找扇叶 + R 标，组成 PowerRune（fanblades[0] 为当前要打的扇叶）。
// 连续丢失 LOSE_MAX 帧才清空历史，短暂遮挡不重置。
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect_24(cv::Mat & bgr_img);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

std::optional<PowerRune> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  void handle_lose();

  YOLO11_BUFF MODE_;
  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
  std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP