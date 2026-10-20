#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace common
{
std::string default_config_path();

class Config
{
public:
  explicit Config(const std::string & path = default_config_path());
  template<typename T>
  T get(const std::string & key) const
  {
    return convert<T>(lookup(key), key);
  }
  template<typename T>
  T get(const std::vector<std::string> & keys) const
  {
    std::string description;
    for (const auto & key : keys) {description += "/" + key;}
    return convert<T>(lookup(keys), description);
  }
  const std::string & path() const {return path_;}
private:
  template<typename T>
  T convert(const YAML::Node & value, const std::string & key) const
  {
    try {
      if (!value.IsScalar()) {throw std::runtime_error("expected a scalar");}
      if constexpr (!std::is_same_v<T, std::string>) {
        if (value.Tag() == "!") {throw std::runtime_error("quoted values are strings");}
      }
      const auto result = value.as<T>();
      if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(result)) {throw std::runtime_error("non-finite value");}
      }
      return result;
    } catch (const std::exception & error) {
      throw std::runtime_error(path_ + ": invalid " + key + ": " + error.what());
    }
  }
  YAML::Node lookup(const std::string & key) const;
  YAML::Node lookup(const std::vector<std::string> & keys) const;
  std::string path_;
  YAML::Node root_;
};
}  // namespace common
