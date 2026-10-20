#include "common/config.hpp"

#include <cstdlib>
#include <set>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace common
{
namespace
{
void validate(const YAML::Node & node)
{
  if (node.IsMap()) {
    std::set<std::string> keys;
    for (const auto & entry : node) {
      if (!entry.first.IsScalar() || !keys.insert(entry.first.as<std::string>()).second) {
        throw std::runtime_error("Configuration keys must be unique scalar strings");
      }
      validate(entry.second);
    }
  } else if (node.IsSequence()) {
    for (const auto & entry : node) {validate(entry);}
  } else if (node.IsNull()) {
    return;
  } else if (!node.IsScalar()) {
    throw std::runtime_error("Unsupported configuration value");
  } else if (node.Tag() != "!") {
    const auto text = node.Scalar();
    if (text == ".nan" || text == ".NaN" || text == ".NAN" ||
      text == ".inf" || text == ".Inf" || text == ".INF" ||
      text == "-.inf" || text == "+.inf")
    {
      throw std::runtime_error("Non-finite configuration value");
    }
  }
}
}

std::string default_config_path()
{
  const char * path = std::getenv("COMMON_CONFIG");
  if (path) {
    if (std::string(path).find_first_not_of(" \t\r\n") == std::string::npos) {
      throw std::runtime_error("COMMON_CONFIG must not be empty");
    }
    return path;
  }
  return ament_index_cpp::get_package_share_directory("common") + "/config/runtime.yaml";
}

Config::Config(const std::string & path) : path_(path)
{
  try {
    root_ = YAML::LoadFile(path_);
    validate(root_);
    if (get<int>("version") != 2) {throw std::runtime_error("expected configuration version: 2");}
    if (!lookup("packages").IsMap()) {throw std::runtime_error("packages must be a mapping");}
  } catch (const std::exception & error) {
    throw std::runtime_error("Cannot load configuration " + path_ + ": " + error.what());
  }
}

YAML::Node Config::lookup(const std::string & key) const
{
  std::istringstream parts(key);
  std::string part;
  std::vector<std::string> keys;
  while (std::getline(parts, part, '.')) {keys.push_back(part);}
  return lookup(keys);
}

YAML::Node Config::lookup(const std::vector<std::string> & keys) const
{
  YAML::Node node = root_;
  std::string traversed;
  for (const auto & part : keys) {
    traversed += "/" + part;
    const YAML::Node current = node;
    if (!current.IsMap() || !current[part]) {
      throw std::runtime_error(path_ + ": missing configuration key " + traversed);
    }
    node.reset(current[part]);
  }
  return node;
}
}  // namespace common
