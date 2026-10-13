#include "live_config.hpp"

#include <unistd.h>

#include <fstream>

#include "tools/logger.hpp"

namespace tools
{
namespace
{
// yaml-cpp 的 Node::operator= 是赋值不是重绑定，逐级下钻只能递归
void put(YAML::Node node, const std::string & key, const YAML::Node & value)
{
  const auto dot = key.find('.');
  if (dot == std::string::npos) {
    node[key] = value;
    return;
  }
  put(node[key.substr(0, dot)], key.substr(dot + 1), value);
}

void write(const std::string & path, const YAML::Node & node)
{
  YAML::Emitter out;
  out << node;
  std::ofstream(path) << out.c_str() << "\n";
}
}  // namespace

LiveConfig::LiveConfig(const std::string & base_path)
: base_path_(base_path),
  merged_path_("/tmp/heavy_live_" + std::to_string(::getpid()) + ".yaml"),
  base_mtime_(std::filesystem::last_write_time(base_path))
{
  merged_ = build();
  write(merged_path_, merged_);
}

LiveConfig::~LiveConfig() { std::filesystem::remove(merged_path_); }

YAML::Node LiveConfig::build() const
{
  auto node = YAML::LoadFile(base_path_);
  for (const auto & [k, v] : overrides_) put(node, k, YAML::Load(v));
  return node;
}

std::set<std::string> LiveConfig::poll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::error_code ec;
  const auto mtime = std::filesystem::last_write_time(base_path_, ec);
  if (ec || (mtime == base_mtime_ && !dirty_)) return {};
  base_mtime_ = mtime;
  dirty_ = false;

  YAML::Node next;
  try {
    next = build();
  } catch (const YAML::Exception & e) {
    // 编辑器保存到一半或写错语法：保留上一版，等下次保存
    logger()->warn("[LiveConfig] {} 解析失败，沿用上一版：{}", base_path_, e.what());
    return {};
  }

  std::set<std::string> changed;
  const YAML::Node & prev = merged_;  // const 下标不会插入空 key
  for (const auto & kv : next) {
    const auto key = kv.first.as<std::string>();
    if (!prev[key] || YAML::Dump(prev[key]) != YAML::Dump(kv.second)) changed.insert(key);
  }
  merged_ = next;
  write(merged_path_, merged_);
  return changed;
}

void LiveConfig::set(const std::string & key, const std::string & value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  overrides_[key] = value;
  dirty_ = true;
}

YAML::Node LiveConfig::node() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return YAML::Clone(merged_);
}

}  // namespace tools
