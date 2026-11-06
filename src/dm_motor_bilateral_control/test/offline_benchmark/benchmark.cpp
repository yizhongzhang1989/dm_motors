// Pure offline causal queue fixture. No ROS runtime, transport, or hardware initialization.
#include "controller.hpp"
#include <dm_motor_control/system_parameters.hpp>
#include "dm_motor_control/simulation/device.hpp"
#include <yaml-cpp/yaml.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace dm_motor_bilateral_control;
using dm_motor::simulation::DeviceParameters;
using dm_motor::simulation::VirtualMotor;
using Tick = int64_t;
constexpr double quantum = 1. / 32000.;
Tick ticks(double s){
  if (!std::isfinite(s) || s < 0) {throw std::runtime_error("Invalid pipeline interval");}
  const auto k = static_cast<Tick>(std::llround(s / quantum));
  if (std::abs(k * quantum - s) > 1e-10) {
    throw std::runtime_error("Off event grid");
  } return k;
}
Tick gate(Tick k, Tick period, Tick phase){
  return k <= phase?phase:phase + ((k - phase + period - 1) / period) * period;
}
bool at(Tick k, Tick period, Tick phase){
  return k >= phase && (k - phase) % period == 0;
}
struct Metadata {
  Tick control{-1};
  std::array<Tick, 2> samples{{-1, -1}};
  double F{0}, C{0};
};
struct Target {
  Tick due;
  size_t axis;
  dm_motor::Command value;
  Metadata meta;
};
struct Pending {
  Tick due, tx;
  size_t axis;
  uint64_t id;
  dm_motor::Frame frame;
  Metadata meta;
  bool stop{false};
};
struct Feedback {
  Tick physical, broker, rx, due;
  size_t axis;
  uint64_t id;
  Sample value;
};
struct Handle {
  double q{0}, v{0};
  bool active{false};
};
// Position and velocity are analytically consistent. Unit-duration ramp primitives.
std::pair<double, double> ramp(double u){
  u = std::clamp(u, 0., 1.);
  return {.5 * (u - std::sin(M_PI * u) / M_PI), .5 * (1 - std::cos(M_PI * u))};
}
Handle trajectory(double t, double speed, bool reciprocal){
  if (t < 1 || t >= 5.5) {
    return {};
  }
  double r = t - 1;
  Handle h;
  h.active = true;
  if (!reciprocal) {
    if (r < .5) {
      auto a = ramp(r / .5);
      h.q = speed * .5 * a.first;
      h.v = speed * a.second;
    }
    else if (r < 3.5) {
      h.q = speed * (.25 + r - .5);
      h.v = speed;
    }
    else if (r < 4) {
      auto a = ramp((r - 3.5) / .5);
      h.q = speed * (3.25 + .5 * ((r - 3.5) / .5 - a.first));
      h.v = speed * (1 - a.second);
    }
    else {
      h.q = speed * 3.5;
    }
  } else {
    // One finite sinusoidal out-and-back action, zero initial/final velocity.
    const double u = std::clamp(r / 4., 0., 1.);
    h.q = speed * 2 / M_PI * (1 - std::cos(2 * M_PI * u));
    h.v = speed * std::sin(2 * M_PI * u);
  }
  return h;
}
int main(int argc, char**argv){
  try {
    if (argc != 4) {
      throw std::runtime_error("Usage: offline_bilateral_benchmark MODELS.json CASE.json TRACE.csv");
    }
    const auto models = YAML::LoadFile(argv[1]), c = YAML::LoadFile(argv[2]);
    auto d = [&](const char*k, double v){
               return c[k]?c[k].as<double>():v;
             };
    auto str = [&](const char*k, std::string v){
                 return c[k]?c[k].as<std::string>():v;
               };
    auto axis = [&](const char*k, size_t i, double v){
                  return !c[k]?v:c[k].IsSequence()?c[k][i].as<double>():c[k].as<double>();
                };
    const auto law = str("law", "shared"), scenario = str("scenario", "drag");
    if (law != "shared" && law != "differential" && law != "disabled") {
      throw std::runtime_error("Unknown benchmark control law");
    }
    const bool disabled = law == "disabled", angle_scenario = scenario == "angle";
    const double speed = d("speed", 1.3), step = d("physics_step", quantum);
    const double friction_fraction = d("friction_fraction", .6);
    if (!std::isfinite(friction_fraction) || friction_fraction < 0 || friction_fraction > 1) {
      throw std::runtime_error("friction_fraction must be finite and in [0,1]");
    }
    const size_t hand_axis = static_cast<size_t>(d("hand_axis", 0));
    if (hand_axis > 1) {
      throw std::runtime_error("Invalid hand axis");
    }
    const Tick period = ticks(.002), brokerperiod = ticks(.001), c2d = ticks(d("c2d", 0)), wire = ticks(d("wire", .001)), back = ticks(d("back", .001)), d2c = ticks(d("d2c", 0));
    const std::array<Tick, 2> phases{{0, ticks(d("stagger", .001))}};
    if (phases[1] >= period) {
      throw std::runtime_error("Invalid stagger");
    }
    const double angle_target = d("angle_degrees", 5) * M_PI / 180;
    const double angle_baseline = d("angle_baseline", 5), angle_transition = d("angle_transition", 2),
      angle_hold = d("angle_hold", 5), angle_disabled_observation = d("angle_disabled_observation", 1);
    Tick angle_begin = 0, angle_stop_due = 0, angle_end = 0;
    if (angle_scenario) {
      if (disabled || angle_baseline <= 0 || angle_disabled_observation <= 0) {
        throw std::runtime_error("Angle scenario requires enabled control and positive quiet/disabled observation durations");
      }
      AngleOffset validation;
      validation.start(angle_target, angle_transition, angle_hold, angle_baseline);
      angle_begin = ticks(angle_baseline);
      angle_stop_due = ticks(angle_baseline + angle_transition + angle_hold);
      if (angle_begin % period || ticks(angle_transition) % period || ticks(angle_hold) % period) {
        throw std::runtime_error("Angle phase boundaries must lie on the 2 ms controller grid");
      }
      angle_end = std::max(gate(angle_stop_due + c2d, period, phases[0]),
        gate(angle_stop_due + c2d, period, phases[1])) + wire + ticks(angle_disabled_observation);
    }
    const double duration = d("duration", angle_scenario ? angle_end * quantum :
      scenario == "drag" || scenario == "reciprocal" ? 8.5 : 5);
    if (angle_scenario && ticks(duration) < angle_end) {
      throw std::runtime_error("Angle duration truncates ACTIVE hold or simulated disabled observation");
    }
    Settings settings;
    settings.control_mode = law == "differential"?ControlMode::DifferentialTorque:ControlMode::SharedReference;
    settings.stiffness = d("stiffness", 50);
    settings.coupling_damping = .4975;
    settings.local_damping = .005;
    settings.direction = static_cast<int>(d("direction", 1));
    settings.max_torque = 0;
    settings.validate();
    std::array<AxisLimits, 2> limits;
    std::array<DeviceParameters, 2> pars;
    std::vector<dm_motor::FrictionCompensator> compensators;
    const auto registry = YAML::LoadFile(str("calibration_file", "bilateral_identification_parameters_20261104.yaml"));
    std::string calibration_namespace = str("calibration_namespace", "manual_identification_replay");
    while (!calibration_namespace.empty() && calibration_namespace.front() == '/') {
      calibration_namespace.erase(calibration_namespace.begin());
    }
    while (!calibration_namespace.empty() && calibration_namespace.back() == '/') {
      calibration_namespace.pop_back();
    }
    const std::string calibration_prefix = "/" + calibration_namespace +
      (calibration_namespace.empty() ? "" : "/");
    for (size_t i = 0; i < 2; ++i) {
      auto&m = pars[i];
      const auto n = models["motor_" + std::to_string(i + 1)];
      m.motor.id = i + 1;
      m.motor.master = 0;
      m.motor.mode = dm_motor::Mode::Mit;
      m.motor.mapping = {n["mapping_position"].as<double>(), n["mapping_velocity"].as<double>(), n["mapping_torque"].as<double>()};
      m.integration_step = step;
      m.physics.inertia = n["inertia"].as<double>();
      m.physics.viscous_friction = n["viscous_friction"].as<double>();
      m.physics.coulomb_friction = n["coulomb_friction"].as<double>();
      m.physics.static_friction = n["static_friction"].as<double>();
      m.physics.stribeck_velocity = n["stribeck_velocity"].as<double>();
      m.physics.friction_negative_scale = n["friction_negative_scale"].as<double>();
      m.physics.gravity_torque = n["gravity_torque"].as<double>();
      m.physics.gravity_offset = n["gravity_offset"].as<double>();
      m.physics.joint_stiffness = n["joint_stiffness"].as<double>();
      m.physics.load_torque = n["load_torque"].as<double>();
      m.physics.max_torque = n["max_torque"].as<double>();
      m.physics.no_load_speed = n["no_load_speed"].as<double>();
      m.physics.constant_torque_speed = n["constant_torque_speed"].as<double>();
      m.physics.torque_time_constant = n["torque_time_constant"].as<double>();
      m.physics.lower_limit = n["lower_limit"].as<double>();
      m.physics.upper_limit = n["upper_limit"].as<double>();
      m.physics.contact_stiffness = n["contact_stiffness"].as<double>();
      m.physics.contact_damping = n["contact_damping"].as<double>();
      m.physics.ambient_temperature = n["ambient_temperature"].as<double>();
      m.physics.heat_capacity = n["heat_capacity"].as<double>();
      m.physics.thermal_resistance = n["thermal_resistance"].as<double>();
      m.physics.winding_resistance = n["winding_resistance"].as<double>();
      m.physics.torque_constant = n["torque_constant"].as<double>();
      m.physics.initial_position = n["initial_position"].as<double>();
      m.physics.initial_velocity = n["initial_velocity"].as<double>();
      m.bus_voltage = n["bus_voltage"].as<double>();
      m.velocity_time_constant = n["velocity_time_constant"].as<double>();
      m.feedback_velocity_time_constant = n["feedback_velocity_time_constant"].as<double>();
      m.feedback_velocity_delay = n["feedback_velocity_delay"].as<double>();
      m.feedback_torque_time_constant = n["feedback_torque_time_constant"].as<double>();
      m.feedback_torque_delay = n["feedback_torque_delay"].as<double>();
      m.velocity_kp = n["velocity_kp"].as<double>();
      m.velocity_ki = n["velocity_ki"].as<double>();
      m.position_kp = n["position_kp"].as<double>();
      m.position_ki = n["position_ki"].as<double>();
      m.anchor_stiffness = n["anchor_stiffness"].as<double>();
      m.physics.friction_model = n["friction_model"].as<std::string>();
      m.feedback_velocity_source = n["feedback_velocity_source"].as<std::string>();
      m.physics.inertia *= axis("inertia_scale", i, 1);
      m.physics.coulomb_friction *= axis("friction_scale", i, 1);
      m.physics.static_friction *= axis("friction_scale", i, 1);
      m.physics.viscous_friction *= axis("friction_scale", i, 1);
      limits[i].mapping = m.motor.mapping;
      limits[i].direction = axis("mapping_direction", i, 1);
      limits[i].validate();
      auto fit = YAML::Clone(registry["motors"][calibration_prefix + "motor_" + std::to_string(i + 1)]["simulation"]["calibrations"]["mechanical"]["fit"]);
      fit["kind"] = "mechanical";
      fit["diagnostics"] = YAML::Load("{}");
      compensators.emplace_back(fit, false);
    }
    std::array<VirtualMotor, 2> motors{{VirtualMotor(pars[0]), VirtualMotor(pars[1])}};
    std::array<Sample, 2> latest{};
    std::array<bool, 2> seen{}, anchored{};
    std::array<double, 2> origins{}, anchor_pm{}, anchor_delta{}, fv{}, friction{}, external{};
    std::array<Tick, 2> sample_ticks{{-1, -1}};
    std::array<uint64_t, 2> sent{}, received{};
    std::array<PeerVelocityFilter, 2> filters;
    std::array<dm_motor::Command, 2> held{}, physical{};
    std::array<Metadata, 2> held_meta{}, physical_meta{};
    std::vector<Target> targets;
    std::vector<Pending> commands;
    std::vector<Feedback> feedbacks;
    Recovery recovery;
    FeedbackPairs pairs;
    AngleOffset angle;
    double angle_applied = 0;
    Tick stop_requested = -1, angle_canceled = -1;
    std::array<Tick, 2> stop_tx{{-1, -1}}, stop_device{{-1, -1}}, last_ordinary_tx{{-1, -1}},
      last_ordinary_device{{-1, -1}};
    std::array<int, 2> stop_status{{-1, -1}};
    bench_control::Result result;
    bool started = false;
    uint64_t clipped = 0, saturated = 0;
    double finish = 0;
    for (size_t i = 0; i < 2; ++i) {
      uint32_t bits = motors[i].register_value(80);
      float pm;
      std::memcpy(&pm, &bits, 4);
      anchor_pm[i] = pm;
      if (!disabled) {
        motors[i].receive(dm_motor::special_frame(i + 1, 0xfc), 0);
      } held[i].p = anchor_pm[i];
    }
    std::ofstream trace(argv[3]), timing(std::string(argv[3]) + ".timing.csv");
    trace << std::setprecision(17);
    timing << std::setprecision(17);
    timing << "axis,id,controller_tick,driver_tick,physical_sample_tick,broker_receive_tick,driver_rx_tick,controller_delivery_tick,source_a_tick,source_b_tick\n";
    trace << "time,qa,qb,va,vb,taua,taub,hand_a,hand_b,handle_q,handle_v,handle_active,relative_error,qa_observed,qb_observed,va_report,vb_report,source_a,source_b,physical_sample_a,physical_sample_b,filtered_va,filtered_vb,pa,pb,da,db,ffa,ffb,comp_a,comp_b,coupling_a,coupling_b,qcentering_a,qcentering_b,held_source_age_a,held_source_age_b,source_skew,physical_skew,recovery,headroom_count,saturated_count";
    if (angle_scenario) {
      trace << ",angle_applied,virtual_error,origin_a,origin_b,angle_active,simulated_stop_requested,simulated_disabled_a,simulated_disabled_b";
    }
    trace << '\n';
    auto deliver_feedback = [&](Tick k){
                              std::stable_sort(feedbacks.begin(), feedbacks.end(), [](const auto&a, const auto&b){
        return a.due < b.due;
      });
                              while (!feedbacks.empty() && feedbacks.front().due <= k) {
                                const auto f = feedbacks.front();
                                feedbacks.erase(feedbacks.begin());
                                const size_t i = f.axis;
                                if (f.id != received[i] + 1 || f.physical > f.broker || f.broker > f.rx || f.rx > f.due || f.due > k || f.value.time > k * quantum) {
                                  throw std::runtime_error("Noncausal/reordered feedback");
                                }
                                received[i] = f.id;
                                if (!seen[i]) {
                                  origins[i] = f.value.position;
                                } seen[i] = true;
                                latest[i] = f.value;
                                sample_ticks[i] = f.physical;
                                pairs.push(i, f.value);
                              }
                            };
    auto apply_commands = [&](Tick k){
                            std::stable_sort(commands.begin(), commands.end(), [](const auto&a, const auto&b){
        return a.due < b.due;
      });
                            while (!commands.empty() && commands.front().due <= k) {
                              const auto p = commands.front();
                              commands.erase(commands.begin());
                              const size_t i = p.axis;
                              if (p.due != k || p.tx > k || p.meta.control > p.tx) {
                                throw std::runtime_error("Noncausal command");
                              }
                              for (Tick s:p.meta.samples) {
                                if (s > p.meta.control && p.meta.control >= 0) {
                                  throw std::runtime_error("Future sample in controller");
                                }
                              }
                              const auto replies = motors[i].receive(p.frame, k * quantum);
                              if (replies.size() != 1) {
                                throw std::runtime_error("Missing MIT feedback");
                              }
                              const auto f = dm_motor::decode_feedback(replies.front(), i + 1, 0, limits[i].mapping);
                              if (!f) {
                                throw std::runtime_error("Invalid feedback codec");
                              }
                              if (p.stop) {
                                stop_device[i] = k;
                                stop_status[i] = motors[i].status();
                                if (f->status != 0 || stop_status[i] != 0) {
                                  throw std::runtime_error("Simulated FD did not disable motor");
                                }
                                physical[i] = {};
                                physical_meta[i] = {};
                                continue;
                              }
                              if (stop_device[i] >= 0) {
                                throw std::runtime_error("Ordinary command delivered after simulated FD");
                              }
                              physical[i] = dm_motor::decode_mit(p.frame, limits[i].mapping);
                              physical_meta[i] = p.meta;
                              last_ordinary_device[i] = k;
                              if (!anchored[i]) {
                                anchor_delta[i] = anchor_pm[i] - f->p;
                                anchored[i] = true;
                              }
                              const Tick broker = gate(k + back, brokerperiod, 0), rx = gate(broker, period, phases[i]), delivery = rx + d2c;
                              Sample s{broker*quantum, limits[i].direction * (f->p + anchor_delta[i]), limits[i].direction * f->v, limits[i].direction * f->t, p.id};
                              feedbacks.push_back({k, broker, rx, delivery, i, p.id, s});
                              timing << i << ',' << p.id << ',' << p.meta.control << ',' << p.tx << ',' << k << ',' << broker << ',' << rx << ',' << delivery << ',' << p.meta.samples[0] << ',' << p.meta.samples[1] << '\n';
                            }
                          };
    std::array<double, 18> sums{};
    uint64_t steady_count = 0;
    for (Tick k = 0; k <= ticks(duration); ++k) {
      const double now = k * quantum;
      finish = now;
      for (auto&m:motors) {
        m.advance(now);
      }
      Handle hand;
      if (scenario == "drag" || scenario == "reciprocal") {
        hand = trajectory(now, speed, scenario == "reciprocal");
      }
      external = {0, 0};
      if (hand.active) {
        const auto&p = motors[hand_axis].state();
        const double q = limits[hand_axis].direction * p.position, v = limits[hand_axis].direction * p.velocity;
        external[hand_axis] = d("handle_stiffness", 15) * (hand.q - q) + d("handle_damping", .05) * (hand.v - v);
      }
      if (scenario == "touch" && now >= 1 && now < 1.2) {
        external[hand_axis] = d("touch_torque", .3);
      }
      if (scenario == "stiffness" && now >= 1 && now < 3) {
        external[0] = d("opposing_torque", .4);
        external[1] = -settings.direction * d("opposing_torque", .4);
      }
      for (size_t i = 0; i < 2; ++i) {
        motors[i].external_torque(limits[i].direction * external[i], .001, now);
      }
      apply_commands(k);
      deliver_feedback(k);
      if (angle_scenario && angle.active() && angle.expired(now) && stop_requested < 0) {
        stop_requested = k;
        targets.clear();
      }
      if (at(k, period, 0) && !disabled && stop_requested < 0 && seen[0] && seen[1]) {
        const bool fresh = pairs.select(now, settings.feedback_age, settings.pair_skew);
        if (pairs.pair()) {
          started = true;
          recovery.update(pairs.age(now), fresh, .002, .002, settings);
        }
        if (started) {
          if (now - std::min(latest[0].time, latest[1].time) > settings.feedback_stop) {
            throw std::runtime_error("Source expired");
          }
          for (size_t i = 0; i < 2; ++i) {
            const double midpoint = std::clamp(latest[i].velocity + limits[i].direction * limits[i].mapping.v / 4095., -limits[i].mapping.v, limits[i].mapping.v);
            fv[i] = filters[i].update(midpoint, latest[i].time);
            friction[i] = compensators[i].step(midpoint, .002, .2, friction_fraction);
          }
          if (angle_scenario && k >= angle_begin) {
            if (!angle.active()) {
              if (!fresh || recovery.scale < 1) {
                throw std::runtime_error("Angle start requires fresh recovered feedback");
              }
              auto reference = latest;
              for (size_t i = 0; i < 2; ++i) {reference[i].velocity = fv[i];}
              validate_angle_offset_path(shared_reference_targets(reference, origins, settings),
                latest, limits, settings, angle_target);
              angle.start(angle_target, angle_transition, angle_hold, now);
            }
            angle_applied = angle.update(now);
          }
          result = bench_control::compute({latest, origins, fv, friction, limits, settings, recovery.scale, angle_applied});
          clipped += result.headroom_clipped;
          for (size_t i = 0; i < 2; ++i) {
            saturated += result.output.protocol_saturated[i];
            targets.push_back({k + c2d, i, result.output.targets[i], {k, sample_ticks, result.output.friction_torque[i], result.coupling[i]}});
          }
        }
      }
      for (size_t i = 0; i < 2; ++i) {
        if (at(k, period, phases[i])) {
          if (stop_requested >= 0) {
            if (stop_tx[i] < 0 && k >= stop_requested + c2d) {
              stop_tx[i] = k;
              commands.push_back({k + wire, k, i, 0, dm_motor::special_frame(i + 1, 0xfd),
                {stop_requested, sample_ticks, 0, 0}, true});
            }
            continue;
          }
          for (auto it = targets.begin(); it != targets.end();) {
            if (it->axis == i && it->due <= k) {
              held[i] = it->value;
              held_meta[i] = it->meta;
              it = targets.erase(it);
            } else {
              ++it;
            }
          }
          commands.push_back({k + wire, k, i, ++sent[i], bench_control::wire(i + 1, held[i], limits[i]), held_meta[i]});
          last_ordinary_tx[i] = k;
        }
      }
      apply_commands(k);
      deliver_feedback(k);
      if (angle_scenario && stop_device[0] >= 0 && stop_device[1] >= 0 && angle_canceled < 0) {
        angle.cancel();
        angle_applied = 0;
        angle_canceled = k;
      }
      std::array<double, 2> q{}, v{}, tau{}, P{}, D{}, FF{}, Q{}, age{};
      for (size_t i = 0; i < 2; ++i) {
        const auto&p = motors[i].state();
        q[i] = limits[i].direction * p.position;
        v[i] = limits[i].direction * p.velocity;
        tau[i] = limits[i].direction * p.motor_torque;
        const bool inactive = disabled || (angle_scenario && motors[i].status() != 1);
        P[i] = inactive?0:limits[i].direction * physical[i].kp * (physical[i].p - motors[i].encoder_position());
        D[i] = inactive?0:limits[i].direction * physical[i].kd * (physical[i].v - p.velocity);
        FF[i] = inactive?0:limits[i].direction * physical[i].t;
        Q[i] = FF[i] - physical_meta[i].F - physical_meta[i].C;
        age[i] = physical_meta[i].samples[0] < 0?-1:now - .5 * (physical_meta[i].samples[0] + physical_meta[i].samples[1]) * quantum;
        if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || std::abs(v[i]) > 30 || std::abs(q[i]) > 10) {
          throw std::runtime_error("Offline runaway limit");
        }
      }
      const bool steady = (angle_scenario ? (k >= angle_stop_due - ticks(1) && k < angle_stop_due) : scenario == "drag"?(now >= 2 && now < 4.2):scenario == "stiffness"?(now >= 2 && now < 3):scenario == "touch"?(now >= 1 && now < 1.2):(now >= duration - 1));
      if (steady) {
        const double dir = settings.direction;
        const std::array<double, 18> values{{external[0] + dir * external[1], v[0], dir*v[1], tau[0] + dir * tau[1], P[0] + dir * P[1], D[0] + dir * D[1], FF[0] + dir * FF[1], physical_meta[0].F + dir * physical_meta[1].F, physical_meta[0].C + dir * physical_meta[1].C, Q[0] + dir * Q[1], age[0], age[1], q[0] - dir * q[1], external[0], external[1], tau[0], tau[1], hand.v}};
        for (size_t j = 0; j < sums.size(); ++j) {
          sums[j] += values[j];
        } ++steady_count;
      }
      if (k % 16 == 0) {
        trace << now;
        auto pairout = [&](const auto&a){
                         trace << ',' << a[0] << ',' << a[1];
                       };
        pairout(q);
        pairout(v);
        pairout(tau);
        pairout(external);
        trace << ',' << hand.q << ',' << hand.v << ',' << hand.active << ',' << q[0] - settings.direction * q[1];
        trace << ',' << latest[0].position << ',' << latest[1].position << ',' << latest[0].velocity << ',' << latest[1].velocity << ',' << latest[0].time << ',' << latest[1].time << ',' << sample_ticks[0] * quantum << ',' << sample_ticks[1] * quantum;
        pairout(fv);
        pairout(P);
        pairout(D);
        pairout(FF);
        trace << ',' << physical_meta[0].F << ',' << physical_meta[1].F << ',' << physical_meta[0].C << ',' << physical_meta[1].C;
        pairout(Q);
        pairout(age);
        trace << ',' << latest[0].time - latest[1].time << ',' << (sample_ticks[0] - sample_ticks[1]) * quantum << ',' << recovery.scale << ',' << clipped << ',' << saturated;
        if (angle_scenario) {
          trace << ',' << angle_applied << ',' << (latest[0].position - origins[0]) -
            settings.direction * (latest[1].position - origins[1]) + angle_applied;
          trace << ',' << origins[0] << ',' << origins[1] << ',' << angle.active() << ',' << (stop_requested >= 0)
            << ',' << (motors[0].status() == 0) << ',' << (motors[1].status() == 0);
        }
        trace << '\n';
      }
    }
    if (angle_scenario) {
      std::ofstream stop(std::string(argv[3]) + ".stop.json");
      stop << std::setprecision(17) << "{\n  \"simulated_only\":true,\n  \"tick_seconds\":" << quantum
           << ",\n  \"requested_tick\":" << stop_requested << ",\n  \"angle_canceled_tick\":" << angle_canceled;
      for (size_t i = 0; i < 2; ++i) {
        stop << ",\n  \"axis_" << i << "\":{\"driver_tx_tick\":" << stop_tx[i]
             << ",\"device_receive_tick\":" << stop_device[i] << ",\"reply_status\":" << stop_status[i]
             << ",\"final_status\":" << static_cast<int>(motors[i].status())
             << ",\"last_ordinary_tx_tick\":" << last_ordinary_tx[i]
             << ",\"last_ordinary_device_tick\":" << last_ordinary_device[i] << '}';
      }
      stop << ",\n  \"disabled_observation\":" << (ticks(duration) - angle_canceled) * quantum << "\n}\n";
      if (stop_requested != angle_stop_due || angle_canceled < 0 || motors[0].status() != 0 ||
        motors[1].status() != 0 || ticks(duration) - angle_canceled < ticks(angle_disabled_observation))
      {
        throw std::runtime_error("Incomplete simulated angle/STOP/disabled sequence");
      }
    }
    std::ofstream means(std::string(argv[3]) + ".integrals.json");
    means << std::setprecision(17) << "{\n  \"step\":" << quantum << ",\n  \"count\":" << steady_count;
    const std::array<std::string, 18> names{{"hand_common", "va", "direction_vb", "motor_common", "P_common", "D_common", "FF_common", "F_common", "C_common", "Q_common", "held_age_a", "held_age_b", "relative_error", "hand_a", "hand_b", "motor_a", "motor_b", "handle_v"}};
    for (size_t j = 0; j < sums.size(); ++j) {
      means << ",\n  \"" << names[j] << "\":" << (steady_count?sums[j] / steady_count:0);
    }
    means << "\n}\n";
    std::cout << "completed duration=" << finish << " feedback=" << received[0] << ',' << received[1] << " headroom=" << clipped << " saturated=" << saturated << "\n";
    return 0;
  }catch(const std::exception&e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
