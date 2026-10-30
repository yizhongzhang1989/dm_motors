#pragma once

#include "common/config.hpp"
#include <dm_motor_interfaces/msg/motor_state.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace dm_motor
{
inline std::string system_parameters_path(const common::Config & config)
{
  if (!config.has("system_parameters_file")) {return {};}
  const auto value = config.get<std::string>("system_parameters_file");
  if (value.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::runtime_error("system_parameters_file must not be empty");
  }
  const std::filesystem::path path(value);
  return (path.is_absolute() ? path :
    std::filesystem::canonical(config.path()).parent_path() / path).string();
}

namespace motor_parameters_detail
{
template<typename T>
T scalar(const YAML::Node & node)
{
  if (!node.IsScalar() || (!std::is_same_v<T, std::string> && node.Tag() == "!")) {
    throw std::runtime_error("Invalid shared motor parameter type");
  }
  const auto value = node.as<T>();
  if constexpr (std::is_floating_point_v<T>) {
    if (!std::isfinite(value)) {throw std::runtime_error("Non-finite shared motor parameter");}
  }
  return value;
}

inline void limit(const YAML::Node & diagnostics, const std::string & actual, const std::string & bound)
{
  const double value = scalar<double>(diagnostics[actual]), maximum = scalar<double>(diagnostics[bound]);
  if (value < 0 || maximum <= 0 || value > maximum) {
    throw std::runtime_error("Unqualified shared calibration: " + actual);
  }
}
}  // namespace motor_parameters_detail

// Same measured-velocity, friction-only law as calibration's FrictionCompensator.
class FrictionCompensator
{
public:
  explicit FrictionCompensator(const YAML::Node & fit, bool require_qualification = true)
  : model_(motor_parameters_detail::scalar<std::string>(fit["model"]))
  {
    using motor_parameters_detail::scalar;
    if (scalar<std::string>(fit["kind"]) != "mechanical" || !fit["parameters"].IsMap() ||
      !fit["units"].IsMap() || !fit["diagnostics"].IsMap() || !fit["domain"].IsMap())
    {
      throw std::runtime_error("Invalid mechanical calibration");
    }
    parameters_ = YAML::Clone(fit["parameters"]);
    for (const auto & item : parameters_) {
      const auto name = scalar<std::string>(item.first);
      scalar<double>(item.second);
      if (scalar<std::string>(fit["units"][name]).empty()) {
        throw std::runtime_error("Calibration parameter is missing its SI unit");
      }
    }
    minimum_ = scalar<double>(fit["domain"]["velocity_min"]);
    maximum_ = scalar<double>(fit["domain"]["velocity_max"]);
    if (minimum_ >= 0 || maximum_ <= 0 || parameter("inertia") < 0) {
      throw std::runtime_error("Invalid mechanical calibration domain/inertia");
    }
    parameter("bias");
    const auto d = fit["diagnostics"];
    for (const auto & section : {d, fit["domain"]}) {
      for (const auto & item : section) {scalar<double>(item.second);}
    }
    if ((require_qualification || d["samples"]) && scalar<double>(d["samples"]) < 3) {
      throw std::runtime_error("Insufficient calibration samples");
    }
    if (parameters_["stiffness"] && parameter("stiffness") < 0) {
      throw std::runtime_error("Invalid mechanical stiffness");
    }
    if (require_qualification) {
      for (const auto & metric : {"rmse", "validation_rmse"}) {
        motor_parameters_detail::limit(d, metric, "rmse_limit");
      }
    }
    if (model_ == "tabulated") {
      const auto knots = fit["friction_table"];
      if (!knots.IsSequence() || knots.size() < 3) {throw std::runtime_error("Invalid friction table");}
      for (const auto & row : knots) {
        if (!row.IsSequence() || row.size() != 4) {throw std::runtime_error("Invalid friction knot");}
        std::array<double, 4> knot;
        for (size_t i = 0; i < 4; ++i) {knot[i] = scalar<double>(row[i]);}
        if (knot[0] <= 0 || knot[2] <= 0 || knot[1] < 0 || knot[3] < 0 ||
          (!table_.empty() && (knot[0] <= table_.back()[0] || knot[2] <= table_.back()[2])))
        {
          throw std::runtime_error("Friction knots must have ordered speeds and nonnegative torques");
        }
        table_.push_back(knot);
      }
      if (require_qualification) {
        for (const auto & metric : {"max_bin_error", "repeatability_max", "ci95_max",
          "static_repeatability_max", "static_ci95_max"})
        {
          const std::string name(metric);
          const std::string bound = name == "max_bin_error" ? "max_bin_error_limit" :
            name.find("ci95") != std::string::npos ? "ci95_limit" : "repeatability_limit";
          motor_parameters_detail::limit(d, name, bound);
        }
        const double required = scalar<double>(d["required_max_speed"]);
        const double validated = scalar<double>(d["validated_max_speed"]);
        if (scalar<double>(d["coverage_complete"]) != 1 || scalar<double>(d["static_complete"]) != 1 ||
          std::abs(required - 2 * std::acos(-1.0)) > 1e-12 || validated < required * 0.998 ||
          std::abs(validated - std::min(maximum_, -minimum_)) > 1e-10 ||
          parameter("static_positive") <= 0 || parameter("static_negative") <= 0)
        {
          throw std::runtime_error("Incomplete precision calibration coverage");
        }
      }
      if (maximum_ != table_.back()[0] || -minimum_ != table_.back()[2]) {
        throw std::runtime_error("Friction table domain does not match measured endpoints");
      }
    } else {
      const std::vector<std::string> models{
        "viscous", "coulomb_viscous", "stribeck", "asymmetric_coulomb_viscous", "asymmetric_stribeck"};
      if (std::find(models.begin(), models.end(), model_) == models.end()) {
        throw std::runtime_error("Unsupported shared friction model: " + model_);
      }
      if (require_qualification) {motor_parameters_detail::limit(d, "condition", "condition_limit");}
      if ((require_qualification || d["condition"]) && scalar<double>(d["condition"]) < 1) {
        throw std::runtime_error("Invalid fit condition");
      }
      for (const auto & suffix : suffixes()) {
        if (parameter("viscous" + suffix) < 0 ||
          (model_ != "viscous" && parameter("coulomb" + suffix) < 0) ||
          (stribeck() && (parameter("static" + suffix) < parameter("coulomb" + suffix) ||
          parameter("stribeck_velocity" + suffix) <= 0)))
        {
          throw std::runtime_error("Invalid friction coefficients");
        }
      }
    }
  }

  void reset() {filtered_ = output_ = 0;}

  double step(double velocity, double dt, double torque_cap)
  {
    if (!std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0 || dt > 0.1 ||
      !std::isfinite(torque_cap) || torque_cap <= 0)
    {
      throw std::runtime_error("Invalid friction compensation input/budget");
    }
    if ((!table_.empty() && std::abs(velocity) < table_.front()[velocity >= 0 ? 0 : 2]) ||
      std::abs(velocity) <= 0.02 || output_ * velocity < 0)
    {
      reset();
      return 0;
    }
    filtered_ += -std::expm1(-dt / 0.01) * (velocity - filtered_);
    if (std::abs(filtered_) <= 0.02) {output_ = 0; return 0;}
    const double v = std::clamp(filtered_, minimum_, maximum_);
    const double weight = std::tanh(std::max(0.0, std::abs(v) - 0.02) / 0.03);
    const double cap = std::min(0.5, torque_cap);
    const double requested = std::clamp(0.8 * weight * friction(v), -cap, cap);
    output_ = std::clamp(output_ + std::clamp(requested - output_, -5 * dt, 5 * dt), -cap, cap);
    return output_;
  }

private:
  double parameter(const std::string & name) const
  {
    return motor_parameters_detail::scalar<double>(parameters_[name]);
  }
  bool stribeck() const {return model_.find("stribeck") != std::string::npos;}
  std::vector<std::string> suffixes() const
  {
    return model_.find("asymmetric") == 0 ?
      std::vector<std::string>{"_positive", "_negative"} : std::vector<std::string>{""};
  }
  double friction(double velocity) const
  {
    if (!table_.empty()) {
      const size_t column = velocity > 0 ? 0 : 2;
      const double speed = std::abs(velocity);
      double torque = table_.back()[column + 1];
      if (speed <= table_.front()[column]) {torque = table_.front()[column + 1];}
      else {
        for (size_t i = 1; i < table_.size(); ++i) {
          if (speed <= table_[i][column]) {
            const auto & a = table_[i - 1];
            const auto & b = table_[i];
            torque = a[column + 1] + (b[column + 1] - a[column + 1]) *
              (speed - a[column]) / (b[column] - a[column]);
            break;
          }
        }
      }
      return std::copysign(torque, velocity);
    }
    const auto suffix = model_.find("asymmetric") == 0 ? (velocity >= 0 ? "_positive" : "_negative") : "";
    double dry = model_ == "viscous" ? 0 : parameter(std::string("coulomb") + suffix);
    if (stribeck()) {
      const double ratio = velocity / parameter(std::string("stribeck_velocity") + suffix);
      dry += (parameter(std::string("static") + suffix) - dry) * std::exp(-ratio * ratio);
    }
    return std::copysign(dry, velocity) + parameter(std::string("viscous") + suffix) * velocity;
  }
  std::string model_;
  YAML::Node parameters_;
  std::vector<std::array<double, 4>> table_;
  double minimum_, maximum_, filtered_{0}, output_{0};
};

inline void check_calibration_identity(
  const YAML::Node & identity, const std::string & name, const dm_motor_interfaces::msg::MotorState & state)
{
  using motor_parameters_detail::scalar;
  if (scalar<std::string>(identity["namespace"]) != name ||
    scalar<int>(identity["motor_id"]) != state.motor_id ||
    scalar<bool>(identity["simulated"]) != state.simulated ||
    scalar<int>(identity["mode"]) != state.mode ||
    scalar<double>(identity["direction"]) != state.direction ||
    scalar<double>(identity["software_offset"]) != state.software_offset ||
    scalar<double>(identity["pmax"]) != state.pmax || scalar<double>(identity["vmax"]) != state.vmax ||
    scalar<double>(identity["tmax"]) != state.tmax)
  {
    throw std::runtime_error(name + ": shared calibration motor identity/coordinates/ranges do not match");
  }
}

class SystemParameters
{
public:
  explicit SystemParameters(const std::string & path) : root_(common::load_yaml(path))
  {
    using motor_parameters_detail::scalar;
    if (scalar<int>(root_["version"]) != 1 || !root_["motors"].IsMap()) {
      throw std::runtime_error(path + ": expected system parameters version 1 and motors mapping");
    }
    for (const auto & motor : root_["motors"]) {
      const auto name = scalar<std::string>(motor.first);
      if (name.empty() || name.front() != '/' || name.back() == '/' ||
        name.find("//") != std::string::npos || !motor.second.IsMap())
      {
        throw std::runtime_error(path + ": invalid motor namespace/description");
      }
      for (const auto & environment : {"hardware", "simulation"}) {
        const auto section = motor.second[environment];
        if (section && (!section.IsMap() || (section["calibrations"] && !section["calibrations"].IsMap()))) {
          throw std::runtime_error(path + ": invalid environment/calibrations mapping");
        }
      }
    }
  }

  std::optional<YAML::Node> motor(const std::string & name) const
  {
    const auto value = root_["motors"][name];
    if (!value) {return std::nullopt;}
    return YAML::Clone(value);
  }

  std::optional<FrictionCompensator> compensator(
    const std::string & name, const dm_motor_interfaces::msg::MotorState & state) const
  {
    using motor_parameters_detail::scalar;
    const auto description = motor(name);
    if (!description) {return std::nullopt;}
    const auto environment = (*description)[state.simulated ? "simulation" : "hardware"];
    if (!environment || !environment["calibrations"]) {return std::nullopt;}
    const auto calibration = environment["calibrations"]["mechanical"];
    if (!calibration) {return std::nullopt;}
    const int schema = scalar<int>(calibration["schema"]);
    if (schema != 2 && schema != 3) {
      throw std::runtime_error(name + ": expected schema-2/3 shared calibration");
    }
    const bool qualified = scalar<bool>(calibration["qualified"]);
    if (!qualified) {
      const auto approval = calibration["approval"];
      if (!approval.IsMap() || !scalar<bool>(approval["user_accepted"]) ||
        (schema == 2 && scalar<std::string>(approval["method"]) != "manual_after_drag"))
      {
        throw std::runtime_error(name + ": candidate requires explicit user acceptance after drag");
      }
      const std::vector<std::string> keys = schema == 3 ?
        std::vector<std::string>{"drag_report", "saved_at"} :
        std::vector<std::string>{"calibration_run", "drag_run", "calibration_sha256", "drag_report_sha256", "saved_at"};
      for (const auto & key : keys) {
        if (scalar<std::string>(approval[key]).empty()) {
          throw std::runtime_error(name + ": incomplete user acceptance evidence");
        }
      }
    }
    auto identity = YAML::Clone(calibration["identity"]);
    auto fit = YAML::Clone(calibration["fit"]);
    if (schema == 3) {
      if (!identity.IsMap() || !fit.IsMap()) {throw std::runtime_error("Invalid compact shared calibration");}
      if (!identity["namespace"]) {identity["namespace"] = name;}
      if (!identity["simulated"]) {identity["simulated"] = state.simulated;}
      if (!fit["kind"]) {fit["kind"] = "mechanical";}
      if (!fit["diagnostics"]) {fit["diagnostics"] = YAML::Node(YAML::NodeType::Map);}
    }
    check_calibration_identity(identity, name, state);
    return FrictionCompensator(fit, schema == 2 && qualified);
  }

private:
  YAML::Node root_;
};
}  // namespace dm_motor
