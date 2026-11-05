#pragma once

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>

namespace hero
{
YAML::Node validate_config(const std::filesystem::path & path);
std::string resolved_config(const std::filesystem::path & path);
}
