#include "dm_motor_control/simulation/physics.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

using dm_motor::simulation::JointPhysics;
using dm_motor::simulation::PhysicsParameters;

namespace
{
PhysicsParameters load(const std::string & path)
{
  PhysicsParameters p;
  const std::map<std::string, double PhysicsParameters::*> fields = {
    {"inertia", &PhysicsParameters::inertia},
    {"viscous_friction", &PhysicsParameters::viscous_friction},
    {"coulomb_friction", &PhysicsParameters::coulomb_friction},
    {"static_friction", &PhysicsParameters::static_friction},
    {"stribeck_velocity", &PhysicsParameters::stribeck_velocity},
    {"friction_negative_scale", &PhysicsParameters::friction_negative_scale},
    {"gravity_torque", &PhysicsParameters::gravity_torque},
    {"gravity_offset", &PhysicsParameters::gravity_offset},
    {"joint_stiffness", &PhysicsParameters::joint_stiffness},
    {"load_torque", &PhysicsParameters::load_torque},
    {"max_torque", &PhysicsParameters::max_torque},
    {"no_load_speed", &PhysicsParameters::no_load_speed},
    {"torque_time_constant", &PhysicsParameters::torque_time_constant},
    {"lower_limit", &PhysicsParameters::lower_limit},
    {"upper_limit", &PhysicsParameters::upper_limit},
    {"contact_stiffness", &PhysicsParameters::contact_stiffness},
    {"contact_damping", &PhysicsParameters::contact_damping},
    {"ambient_temperature", &PhysicsParameters::ambient_temperature},
    {"heat_capacity", &PhysicsParameters::heat_capacity},
    {"thermal_resistance", &PhysicsParameters::thermal_resistance},
    {"winding_resistance", &PhysicsParameters::winding_resistance},
    {"torque_constant", &PhysicsParameters::torque_constant},
    {"initial_position", &PhysicsParameters::initial_position},
    {"initial_velocity", &PhysicsParameters::initial_velocity},
  };
  const auto document = YAML::LoadFile(path);
  if (!document.IsMap()) {throw std::invalid_argument("Physics settings must be a mapping");}
  std::set<std::string> seen;
  for (const auto & item : document) {
    const auto key = item.first.as<std::string>();
    if (!seen.insert(key).second) {throw std::invalid_argument("Duplicate physics parameter: " + key);}
    if (key == "friction_model") {p.friction_model = item.second.as<std::string>();}
    else {
      const auto field = fields.find(key);
      if (field == fields.end()) {throw std::invalid_argument("Unknown physics parameter: " + key);}
      p.*(field->second) = item.second.as<double>();
    }
  }
  p.validate();
  return p;
}

void publish(const JointPhysics & plant, double time, double command, double external, bool saturated)
{
  const auto & s = plant.state();
  const auto & p = plant.parameters();
  const double current = s.motor_torque / p.torque_constant;
  std::cout << std::setprecision(17) << time << ' ' << s.position << ' ' << s.velocity << ' '
            << s.motor_torque << ' ' << command << ' ' << external << ' ' << s.rotor_temperature << ' '
            << current << ' ' << p.ambient_temperature << ' '
            << current * current * p.winding_resistance << ' ' << s.motor_torque << ' '
            << (saturated ? 1 : 0) << std::endl;
}
}

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) {throw std::invalid_argument("Usage: calibration_simulator PHYSICS.yaml");}
    JointPhysics plant(load(argv[1]));
    double time = 0;
    publish(plant, time, 0, 0, false);
    std::string line;
    while (std::getline(std::cin, line)) {
      std::istringstream input(line);
      double dt, command, external;
      double position = 0, velocity = 0, kp = 0, kd = 0;
      int enabled;
      std::string extra, first;
      input >> first;
      const bool mit = first == "mit";
      if (!mit) {input.clear(); input.str(line);}
      const bool parsed = mit ?
        static_cast<bool>(input >> dt >> position >> velocity >> kp >> kd >> command >> external >> enabled) :
        static_cast<bool>(input >> dt >> command >> external >> enabled);
      if (!parsed || (input >> extra) ||
        !std::isfinite(dt) || dt <= 0 || dt > 0.1 || (enabled != 0 && enabled != 1))
      {
        throw std::invalid_argument("Expected dt (0,0.1], torque, external torque, enabled (0/1)");
      }
      const int steps = static_cast<int>(std::ceil(dt / 0.00025));
      if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(kp) ||
        !std::isfinite(kd) || kp < 0 || kp > 500 || kd < 0 || kd > 1)
      {
        throw std::invalid_argument("Invalid native MIT servo request");
      }
      bool saturated = false;
      double requested = command;
      for (int step = 0; step < steps; ++step) {
        const auto & p = plant.parameters();
        const double speed = plant.state().velocity;
        requested = command + kp * (position - plant.state().position) + kd * (velocity - speed);
        const double limit = p.max_torque *
          (requested * speed > 0 ? std::max(0.0, 1 - std::abs(speed) / p.no_load_speed) : 1);
        saturated = saturated || (enabled && std::abs(requested) > limit);
        plant.step(dt / steps, requested, external, enabled != 0);
      }
      time += dt;
      publish(plant, time, requested, external, saturated);
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "calibration_simulator: " << error.what() << '\n';
    return 1;
  }
}
