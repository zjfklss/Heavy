#ifndef TOOLS__LIVE_CONFIG_HPP
#define TOOLS__LIVE_CONFIG_HPP

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <string>

namespace tools
{

// 运行时改参（对应旧 ROS node_params.yaml + 动态参数）。
// 同济各模块只认 yaml 路径，所以把「磁盘 yaml + 面板覆盖」合成一份临时 yaml，
// 主循环 poll() 到变化后用 path() 重建便宜的对象（Aimer/Shooter/Tracker…）。
// 面板覆盖只在本次运行有效；想永久生效请改磁盘 yaml。
class LiveConfig
{
public:
  explicit LiveConfig(const std::string & base_path);
  ~LiveConfig();

  const std::string & path() const { return merged_path_; }
  const std::string & base_path() const { return base_path_; }

  // 主线程调：磁盘文件被保存或面板改过 → 重写合并文件，返回变化的顶层 key
  std::set<std::string> poll();

  // 任意线程：key 支持 a.b 写嵌套；value 按 yaml 字面量解析（"1.5"/"true"/"red"）
  void set(const std::string & key, const std::string & value);

  YAML::Node node() const;

private:
  std::string base_path_;
  std::string merged_path_;
  std::filesystem::file_time_type base_mtime_;
  YAML::Node merged_;
  std::map<std::string, std::string> overrides_;
  bool dirty_ = false;
  mutable std::mutex mutex_;

  YAML::Node build() const;
};

}  // namespace tools

#endif  // TOOLS__LIVE_CONFIG_HPP
