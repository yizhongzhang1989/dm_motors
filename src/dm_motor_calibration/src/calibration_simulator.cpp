#include "dm_motor_control/simulation/physics.hpp"
#include "dm_motor_control/simulation/feedback.hpp"

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
using dm_motor::simulation::FeedbackDynamics;
using dm_motor::simulation::FeedbackParameters;

namespace
{
struct SimulationParameters
{
  PhysicsParameters physics;
  FeedbackParameters feedback;
  double integration_step{0.00025};  // Preserve the original calibration substep default.
};

SimulationParameters load(const std::string & path)
{
  SimulationParameters settings;
  auto & p = settings.physics;
  const std::map<std::string, double PhysicsParameters::*> fields = {
    {"inertia", &PhysicsParameters::inertia},
    {"viscous_friction", &PhysicsParameters::viscous_friction},
    {"coulomb_friction", &PhysicsParameters::coulomb_friction},
    {"static_friction", &PhysicsParameters::static_friction},
    {"stribeck_velocity", &PhysicsParameters::stribeck_velocity},
    {"friction_negative_scale", &PhysicsParameters::friction_negative_scale},
    {"friction_displacement_scale", &PhysicsParameters::friction_displacement_scale},
    {"initial_bristle_displacement", &PhysicsParameters::initial_bristle_displacement},
    {"gravity_torque", &PhysicsParameters::gravity_torque},
    {"gravity_offset", &PhysicsParameters::gravity_offset},
    {"joint_stiffness", &PhysicsParameters::joint_stiffness},
    {"load_torque", &PhysicsParameters::load_torque},
    {"max_torque", &PhysicsParameters::max_torque},
    {"constant_torque_speed", &PhysicsParameters::constant_torque_speed},
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
  const std::map<std::string, double FeedbackParameters::*> feedback_fields = {
    {"velocity_time_constant", &FeedbackParameters::velocity_time_constant},
    {"feedback_velocity_time_constant", &FeedbackParameters::feedback_velocity_time_constant},
    {"feedback_velocity_delay", &FeedbackParameters::feedback_velocity_delay},
    {"feedback_torque_time_constant", &FeedbackParameters::feedback_torque_time_constant},
    {"feedback_torque_delay", &FeedbackParameters::feedback_torque_delay},
  };
  const auto document = YAML::LoadFile(path);
  if (!document.IsMap()) {throw std::invalid_argument("Physics settings must be a mapping");}
  std::set<std::string> seen;
  for (const auto & item : document) {
    const auto key = item.first.as<std::string>();
    if (!seen.insert(key).second) {throw std::invalid_argument("Duplicate physics parameter: " + key);}
    if (key == "friction_model") {p.friction_model = item.second.as<std::string>();}
    else if (key == "integration_step") {settings.integration_step = item.second.as<double>();}
    else if (key == "feedback_velocity_source") {settings.feedback.feedback_velocity_source = item.second.as<std::string>();}
    else if (feedback_fields.count(key)) {settings.feedback.*(feedback_fields.at(key)) = item.second.as<double>();}
    else {
      const auto field = fields.find(key);
      if (field == fields.end()) {throw std::invalid_argument("Unknown physics parameter: " + key);}
      p.*(field->second) = item.second.as<double>();
    }
  }
  p.validate();
  settings.feedback.validate();
  if (!std::isfinite(settings.integration_step) || settings.integration_step < 1e-5 || settings.integration_step > 0.001) {
    throw std::invalid_argument("Integration step must be finite and in [0.00001,0.001] seconds");
  }
  return settings;
}

void publish(const JointPhysics & plant, const FeedbackDynamics & feedback,
  double time, double command, double external, bool saturated)
{
  const auto & s = plant.state();
  const auto & p = plant.parameters();
  const double current = s.motor_torque / p.torque_constant;
  std::cout << std::setprecision(17) << time << ' ' << s.position << ' ' << feedback.velocity(time) << ' '
            << feedback.torque(time) << ' ' << command << ' ' << external << ' ' << s.rotor_temperature << ' '
            << current << ' ' << p.ambient_temperature << ' '
            << current * current * p.winding_resistance << ' ' << s.motor_torque << ' '
            << (saturated ? 1 : 0) << ' ' << s.velocity << ' ' << s.motor_torque << ' '
            << feedback.internal_velocity() << std::endl;
}
}

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) {throw std::invalid_argument("Usage: calibration_simulator PHYSICS.yaml");}
    const auto settings = load(argv[1]);
    JointPhysics plant(settings.physics);
    FeedbackDynamics feedback(settings.feedback, settings.physics.initial_velocity);
    double time = 0;
    publish(plant, feedback, time, 0, 0, false);
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
        !std::isfinite(dt) || dt <= 0 || dt > 0.1 || !std::isfinite(command) ||
        !std::isfinite(external) || (enabled != 0 && enabled != 1))
      {
        throw std::invalid_argument("Expected dt (0,0.1], torque, external torque, enabled (0/1)");
      }
      const int steps = static_cast<int>(std::ceil(dt / settings.integration_step));
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
        requested = command + kp * (position - plant.state().position) +
          kd * (velocity - feedback.internal_velocity());
        double limit = p.max_torque;
        if (requested * speed > 0) {
          if (p.constant_torque_speed == 0) {limit *= std::max(0.0, 1 - std::abs(speed) / p.no_load_speed);}
          else if (std::abs(speed) > p.constant_torque_speed) {
            limit *= std::max(0.0, 1 - (std::abs(speed) - p.constant_torque_speed) /
                (p.no_load_speed - p.constant_torque_speed));
          }
        }
        saturated = saturated || (enabled && std::abs(requested) > limit);
        plant.step(dt / steps, requested, external, enabled != 0);
        feedback.advance(dt / steps, time + (step + 1) * (dt / steps),
          plant.state().velocity, plant.state().motor_torque);
      }
      time += dt;
      publish(plant, feedback, time, requested, external, saturated);
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "calibration_simulator: " << error.what() << '\n';
    return 1;
  }
}
