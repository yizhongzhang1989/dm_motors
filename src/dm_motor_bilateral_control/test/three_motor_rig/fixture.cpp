// Pure offline fixture: production plant, wire codec and bilateral controller.
#include "coupling.hpp"
#include "feedback_runtime.hpp"
#include "test_feedback_runtime.hpp"
#include "../offline_benchmark/controller.hpp"
#include <dm_motor_control/simulation/device.hpp>
#include <dm_motor_control/system_parameters.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace dm_motor_bilateral_control;
using dm_motor::simulation::DeviceParameters;
using dm_motor::simulation::VirtualMotor;
using Tick = int64_t;
constexpr double dt = 1. / 32000.;
constexpr Tick period = 64;

struct ReferenceFile
{
  std::vector<std::pair<double, double>> rows;
  explicit ReferenceFile(const std::string & path)
  {
    std::ifstream input(path);
    std::string line;
    if (!input || !std::getline(input, line) || line != "time,q,v") {
      throw std::invalid_argument("Reference file requires exact time,q,v CSV header");
    }
    while (std::getline(input, line)) {
      double time, q, v; char comma1, comma2;
      std::istringstream row(line);
      if (!(row >> time >> comma1 >> q >> comma2 >> v) || comma1 != ',' || comma2 != ',' ||
        !(row >> std::ws).eof() || !std::isfinite(time) || !std::isfinite(q) || !std::isfinite(v) ||
        std::abs(time - rows.size() * .002) > 1e-10 ||
        std::abs(q) >= 12.4 || std::abs(v) > 13.3 || time > 360 ||
        (!rows.empty() && std::abs(q - rows.back().first) > 13.3 * .002 + 1e-9))
      {throw std::invalid_argument("Invalid finite500Hz reference time, native envelope or position increment");}
      rows.emplace_back(q, v);
    }
    if (rows.size() < 1601 || std::abs(rows.front().second) > 1e-12 ||
      std::abs(rows.back().second) > 1e-12)
    {throw std::invalid_argument("Reference must last at least3.2s and start/end with zero velocity");}
  }
  double duration() const {return (rows.size() - 1) * .002;}
  std::pair<double, double> at(double time) const
  {
    const auto index = std::min(rows.size() - 1,
      static_cast<size_t>(std::floor(time / .002 + 1e-9)));
    return {rows[index].first - rows.front().first, rows[index].second};
  }
};

Tick event_ticks(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0 || seconds > .02) {
    throw std::invalid_argument("Pipeline intervals must be finite and in [0,.02] seconds");
  }
  const auto count = static_cast<Tick>(std::llround(seconds / dt));
  if (std::abs(count * dt - seconds) > 1e-12) {
    throw std::invalid_argument("Pipeline interval is not on the 31.25us grid");
  }
  return count;
}

DeviceParameters parameters(const YAML::Node & n, uint8_t id)
{
  DeviceParameters m;
  m.motor.id = id;
  m.motor.master = 0;
  m.motor.mode = dm_motor::Mode::Mit;
  m.motor.mapping = {n["mapping_position"].as<double>(), n["mapping_velocity"].as<double>(),
    n["mapping_torque"].as<double>()};
  m.integration_step = dt;
#define PHYSICS(field) m.physics.field = n[#field].as<double>()
  PHYSICS(inertia); PHYSICS(viscous_friction); PHYSICS(coulomb_friction);
  PHYSICS(static_friction); PHYSICS(stribeck_velocity); PHYSICS(friction_negative_scale);
  PHYSICS(gravity_torque); PHYSICS(gravity_offset); PHYSICS(joint_stiffness);
  PHYSICS(load_torque); PHYSICS(max_torque); PHYSICS(no_load_speed);
  PHYSICS(constant_torque_speed); PHYSICS(torque_time_constant);
  PHYSICS(lower_limit); PHYSICS(upper_limit); PHYSICS(contact_stiffness);
  PHYSICS(contact_damping); PHYSICS(ambient_temperature); PHYSICS(heat_capacity);
  PHYSICS(thermal_resistance); PHYSICS(winding_resistance); PHYSICS(torque_constant);
  PHYSICS(initial_position); PHYSICS(initial_velocity);
  if (n["friction_displacement_scale"]) {PHYSICS(friction_displacement_scale);}
  if (n["initial_bristle_displacement"]) {PHYSICS(initial_bristle_displacement);}
#undef PHYSICS
#define DEVICE(field) m.field = n[#field].as<double>()
  DEVICE(bus_voltage); DEVICE(velocity_time_constant); DEVICE(feedback_velocity_time_constant);
  DEVICE(feedback_velocity_delay); DEVICE(feedback_torque_time_constant);
  DEVICE(feedback_torque_delay); DEVICE(velocity_kp); DEVICE(velocity_ki);
  DEVICE(position_kp); DEVICE(position_ki); DEVICE(anchor_stiffness);
#undef DEVICE
  m.physics.friction_model = n["friction_model"].as<std::string>();
  m.feedback_velocity_source = n["feedback_velocity_source"].as<std::string>();
  return m;
}

// Matches prepared/physical_drag_v2.py: .3 s baseline, finite positive/negative sweeps.
double ramp_time(double speed) {return std::max(.25, speed / 15.);}
double leg_time(double speed) {return 2 * ramp_time(speed) + .6;}
double travel(double speed) {return speed * (ramp_time(speed) + .6);}
std::pair<double, double> outward(double t, double speed)
{
  const double pi = std::acos(-1.0), ramp = ramp_time(speed), cruise = .6;
  if (t <= 0) {return {0, 0};}
  if (t < ramp) {
    const double u = t / ramp;
    return {speed * ramp * .5 * (u - std::sin(pi * u) / pi),
      speed * .5 * (1 - std::cos(pi * u))};
  }
  if (t < ramp + cruise) {return {speed * (ramp / 2 + t - ramp), speed};}
  if (t < 2 * ramp + cruise) {
    const double u = (t - ramp - cruise) / ramp;
    return {speed * (ramp / 2 + cruise + ramp * .5 * (u + std::sin(pi * u) / pi)),
      speed * .5 * (1 + std::cos(pi * u))};
  }
  return {speed * (ramp + cruise), 0};
}

std::pair<double, double> trajectory(double time, double speed)
{
  if (time < .55 + leg_time(speed)) {return outward(time - .3, speed);}
  const auto back = outward(time - .55 - leg_time(speed), speed);
  return {travel(speed) - back.first, -back.second};
}

std::string bytes(const dm_motor::Frame & frame)
{
  std::ostringstream s;
  s << std::hex << std::setfill('0');
  for (size_t i = 0; i < frame.length; ++i) {s << std::setw(2) << int(frame.data[i]);}
  return s.str();
}

struct PendingCommand
{
  Tick due, sent, control;
  size_t axis;
  uint64_t sequence;
  std::array<Tick, 2> sources;
  dm_motor::Frame frame;
  double friction, coupling;
};
struct PendingFeedback
{
  Tick due, sampled, received, nominal_received;
  size_t axis;
  uint64_t sequence;
  dm_motor::Frame frame;
  dm_motor::MotorSnapshot state{};
};

int main(int argc, char ** argv)
{
  try {
    if (argc == 2 && std::string(argv[1]) == "--test-feedback-wrap") {
      return three_motor_rig::check_feedback_wraps();
    }
    if (argc == 2 && std::string(argv[1]) == "--test-feedback-rx") {
      return three_motor_rig::check_feedback_batches();
    }
    if (argc != 4) {
      throw std::runtime_error("Usage: three_motor_fixture MODELS.json CASE.json TRACE.csv");
    }
    const auto models = YAML::LoadFile(argv[1]), c = YAML::LoadFile(argv[2]);
    auto number = [&](const char * name, double fallback) {
        return c[name] ? c[name].as<double>() : fallback;
      };
    const double speed = number("speed", .3), fraction = number("friction_fraction", .6);
    const double c_kp = number("c_kp", 20), c_kd = number("c_kd", .6);
    const double c_bandwidth = number("c_velocity_bandwidth", 100);
    const std::string position_mode = c["position_feedback_mode"] ?
      c["position_feedback_mode"].as<std::string>() : "latest";
    if (!std::isfinite(c_kp) || c_kp < 0 || c_kp > 500 ||
      !std::isfinite(c_kd) || c_kd < 0 || c_kd > dm_motor::max_kd ||
      !std::isfinite(c_bandwidth) || c_bandwidth <= 0 ||
      (position_mode != "latest" && position_mode != "common_time" && position_mode != "synchronized"))
    {throw std::invalid_argument("Invalid C servo gains, bandwidth or position feedback mode");}
    const Tick command_delay = event_ticks(number("command_delay", .001));
    const Tick feedback_delay = event_ticks(number("feedback_delay", .001));
    const Tick b_phase = event_ticks(number("b_phase", .001));
    const Tick controller_phase = event_ticks(number("controller_phase", 0));
    const int batch_axis = c["rx_batch_axis"] ? c["rx_batch_axis"].as<int>() : 0;
    const Tick batch_window = event_ticks(number("rx_batch_window", .010));
    const Tick batch_gap = event_ticks(number("rx_batch_gap", .010));
    const double batch_at_seconds = number("rx_batch_at", 1.501);
    if (batch_axis < 0 || batch_axis > 2 || !std::isfinite(batch_at_seconds) ||
      batch_at_seconds < 0 || batch_at_seconds > 360 || controller_phase >= period)
    {throw std::invalid_argument("Invalid receive collection axis, time or controller phase");}
    const Tick batch_at = static_cast<Tick>(std::llround(batch_at_seconds / dt));
    if (std::abs(batch_at * dt - batch_at_seconds) > 1e-12 ||
      (batch_axis && (batch_window == 0 || batch_gap == 0 || batch_gap > batch_window ||
      batch_window - batch_gap >= period || batch_at <= batch_window ||
      (batch_at - controller_phase) % period != 0 ||
      (batch_at - command_delay - feedback_delay - (batch_axis == 2 ? b_phase : 0)) % period != 0)))
    {throw std::invalid_argument("Receive batch must preserve order and end on the selected RX/control phase");}
    const int pause_axis = c["feedback_pause_axis"] ? c["feedback_pause_axis"].as<int>() : 0;
    const double pause_start_seconds = number("feedback_pause_start", 4.21);
    const Tick pause_duration = event_ticks(number("feedback_pause_duration", 0));
    if (pause_axis < 0 || pause_axis > 2 || !std::isfinite(pause_start_seconds) ||
      pause_start_seconds < 0 || pause_start_seconds > 60)
    {throw std::invalid_argument("Invalid single-axis application feedback pause");}
    const Tick pause_start = static_cast<Tick>(std::llround(pause_start_seconds / dt));
    if (std::abs(pause_start * dt - pause_start_seconds) > 1e-12) {
      throw std::invalid_argument("Feedback pause start is not on the physics grid");
    }
    if (command_delay == 0 || feedback_delay == 0 || b_phase >= period) {
      throw std::invalid_argument("Delays must be positive; B phase must be less than 2ms");
    }
    const std::string scenario = c["scenario"] ? c["scenario"].as<std::string>() : "trajectory";
    if (scenario != "trajectory" && scenario != "touch" &&
      scenario != "touch_pair" && scenario != "release_pair")
    {
      throw std::invalid_argument("Unknown fixture scenario");
    }
    std::optional<ReferenceFile> reference_file;
    if (c["reference_file"]) {
      if (scenario != "trajectory") {throw std::invalid_argument("Reference file requires trajectory scenario");}
      reference_file.emplace(c["reference_file"].as<std::string>());
    }
    const double trace_rate = number("trace_rate", 2000);
    if (trace_rate != 500 && trace_rate != 1000 && trace_rate != 2000) {
      throw std::invalid_argument("Trace rate must be500,1000 or2000Hz");
    }
    const Tick trace_period = static_cast<Tick>(32000 / trace_rate);
    const bool touching = scenario == "touch" || scenario == "touch_pair";
    const double probe_torque = number("probe_torque", number("probe_torque3", .55)), probe_start = number("probe_start", .5),
      probe_duration = number("probe_duration", .08);
    if (!std::isfinite(probe_torque) || std::abs(probe_torque) > 1 ||
      !std::isfinite(probe_start) || probe_start < .3 ||
      !std::isfinite(probe_duration) || probe_duration <= 0 || probe_duration > .2)
    {throw std::invalid_argument("Invalid finite C-axis touch probe");}
    auto reference = [&](double time) {
        return reference_file ? reference_file->at(time) :
          touching ? std::pair<double, double>{0, 0} : trajectory(time, speed);
      };
    const double duration = number("duration", reference_file ? reference_file->duration() : 2 * leg_time(speed) + 1.05);
    if (reference_file && std::abs(duration - reference_file->duration()) > 1e-10) {
      throw std::invalid_argument("Case duration differs from complete reference file");
    }
    if (batch_axis && batch_at_seconds >= duration - .02) {
      throw std::invalid_argument("Receive batch must drain before the case ends");
    }
    const double release_time = number("release_time", .6 + ramp_time(speed));
    if (!std::isfinite(release_time) || release_time < .3 || release_time >= duration - .2) {
      throw std::invalid_argument("Invalid finite A/B release time");
    }
    if (!std::isfinite(speed) || speed <= 0 || travel(speed) / 2 >= 12.5 ||
      !std::isfinite(fraction) || fraction < 0 || fraction > 1 ||
      !std::isfinite(duration) || duration < 3.2 || duration > (reference_file ? 360 : 60))
    {throw std::runtime_error("Invalid speed, friction fraction or duration");}
    Settings settings;
    settings.control_mode = ControlMode::DifferentialTorque;
    const std::string coupling_mode = c["coupling_mode"] ? c["coupling_mode"].as<std::string>() : "bilateral";
    if (coupling_mode == "a_leads_b") {settings.coupling_mode = CouplingMode::ALeadsB;}
    else if (coupling_mode != "bilateral") {
      throw std::invalid_argument("coupling_mode must be bilateral or a_leads_b");
    }
    settings.stiffness = number("stiffness", 50);
    settings.coupling_damping = number("coupling_damping", .4975);
    settings.local_damping = number("local_damping", .005);
    settings.friction_torque_limit = number("friction_torque_limit", .2);
    settings.friction_compensation_fraction = fraction;
    settings.extra_viscous = {number("extra_viscous_1", 0), number("extra_viscous_2", 0)};
    settings.extra_viscous_velocity_limit = number("extra_viscous_velocity_limit", 0);
    settings.direction = 1;
    settings.validate();
    three_motor_rig::Coupling link;
    link.stiffness = number("shaft_stiffness", 40);
    link.damping = number("shaft_damping", .02);
    link.half_gap = number("shaft_half_gap", 0);
    const bool independent_c = static_cast<bool>(models["motor_3"]);
    std::array<DeviceParameters, 3> pars{{parameters(models["motor_1"], 1),
      parameters(models["motor_2"], 2),
      parameters(independent_c ? models["motor_3"] : models["motor_1"], 3)}};
    // Without an explicit C model, copying A remains a scenario assumption.
    pars[2].physics.initial_position = reference_file ? reference_file->rows.front().first : -travel(speed) / 2;
    pars[0].physics.initial_position = pars[1].physics.initial_position = -pars[2].physics.initial_position;
    for (auto & p : pars) {p.physics.initial_velocity = 0;}
    link.offset = pars[0].physics.initial_position + pars[2].physics.initial_position;
    link.validate();
    std::array<VirtualMotor, 3> motors{{VirtualMotor(pars[0]), VirtualMotor(pars[1]),
      VirtualMotor(pars[2])}};
    // Exercise the real simulated register protocol while disabled. Register32 currently
    // stores a value only: it does NOT identify or change firmware velocity dynamics.
    dm_motor::Frame bandwidth_frame;
    bandwidth_frame.id = 0x7ff; bandwidth_frame.length = 8;
    bandwidth_frame.data[0] = 3; bandwidth_frame.data[2] = 0x55;
    bandwidth_frame.data[3] = 32;
    const float bandwidth_float = static_cast<float>(c_bandwidth);
    uint32_t bandwidth_raw;
    std::memcpy(&bandwidth_raw, &bandwidth_float, sizeof(bandwidth_raw));
    for (unsigned i = 0; i < 4; ++i) {
      bandwidth_frame.data[4 + i] = (bandwidth_raw >> (8 * i)) & 255;
    }
    motors[2].receive(bandwidth_frame, 0);
    if (motors[2].register_value(32) != bandwidth_raw) {
      throw std::runtime_error("C bandwidth register readback mismatch");
    }
    three_motor_rig::FeedbackRuntime feedback_runtime(motors);
    std::array<AxisLimits, 2> limits;
    std::vector<dm_motor::FrictionCompensator> compensators;
    const auto registry = YAML::LoadFile(c["calibration_file"].as<std::string>());
    std::string ns = c["calibration_namespace"].as<std::string>();
    while (!ns.empty() && ns.front() == '/') {ns.erase(ns.begin());}
    while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
    const std::string prefix = "/" + ns + (ns.empty() ? "" : "/");
    for (size_t i = 0; i < 2; ++i) {
      limits[i].mapping = pars[i].motor.mapping;
      limits[i].validate();
      auto fit = YAML::Clone(registry["motors"][prefix + "motor_" + std::to_string(i + 1)]
        ["simulation"]["calibrations"]["mechanical"]["fit"]);
      fit["kind"] = "mechanical";
      fit["diagnostics"] = YAML::Load("{}");
      compensators.emplace_back(fit, false);
    }
    std::array<Sample, 3> latest{};
    std::array<double, 3> native_position{};
    std::array<int64_t, 3> position_wraps{};
    FeedbackPairs pairs;
    bool synchronization_started = false;
    std::array<std::deque<Sample>, 2> decoded_history;
    std::array<double, 2> control_position{}, control_position_time{};
    std::array<bool, 3> seen{};
    std::array<double, 2> origins{}, filtered{}, friction{};
    std::array<PeerVelocityFilter, 2> filters;
    std::array<Tick, 3> sampled{{-1, -1, -1}}, applied_at{{-1, -1, -1}};
    std::array<Tick, 3> delivered_at{{-1, -1, -1}};
    std::array<uint64_t, 3> sequences{};
    std::array<uint64_t, 3> received_sequences{}, received_counts{};
    std::array<dm_motor::Command, 3> targets{}, applied{};
    std::array<double, 3> target_friction{}, target_coupling{}, applied_friction{}, applied_coupling{};
    std::array<Tick, 3> target_control{{-1, -1, -1}};
    std::array<std::array<Tick, 2>, 3> target_sources{{{{-1, -1}}, {{-1, -1}}, {{-1, -1}}}};
    std::deque<PendingCommand> commands;
    std::deque<PendingFeedback> feedbacks;
    std::deque<PendingFeedback> transport_feedbacks;
    bench_control::Result result;
    for (size_t i = 0; i < 3; ++i) {
      targets[i].p = pars[i].physics.initial_position;
    }
    std::ofstream trace(argv[3]), events(std::string(argv[3]) + ".events.csv"),
      receptions(std::string(argv[3]) + ".rx.csv"), controls(std::string(argv[3]) + ".control.csv");
    if (!trace || !events || !receptions || !controls) {throw std::runtime_error("Cannot write output");}
    trace << std::setprecision(17);
    controls << std::setprecision(17);
    events << "axis,sequence,controller_tick,send_tick,device_sample_tick,feedback_delivery_tick,source_a_tick,source_b_tick,command_hex,feedback_hex,application_delivery_tick,nominal_feedback_tick\n";
    receptions << "axis,sequence,device_sample_tick,nominal_feedback_tick,transport_receive_tick,application_delivery_tick,bus_feedback_count,feedback_hex\n";
    controls << "time,available,common_time,latest_time1,latest_time2,position1,position2,bracket_gap1,bracket_gap2,latest_age1,latest_age2,lower_age1,upper_age1,lower_age2,upper_age2,reason\n";
    trace << "time,q3_reference,v3_reference,probe_torque1,probe_torque3,coupling_active,coupling_delta,coupling_extension,coupling_torque1,coupling_torque3,coupling_energy,coupling_dissipation";
    for (size_t i = 1; i <= 3; ++i) {
      for (const auto * field : {"q_truth", "v_truth", "t_truth", "q_reported", "v_reported",
        "t_reported", "sample_time", "command_time", "command_p", "command_v", "command_kp",
        "command_kd", "command_t", "plant_friction", "external_torque", "status",
        "native_position", "position_wraps", "mos_temperature", "rotor_temperature"})
      {trace << ',' << field << i;}
    }
    trace << ",filtered_v1,filtered_v2,friction_requested1,friction_requested2,friction_applied1,friction_applied2,bilateral_torque1,bilateral_torque2,relative_error,headroom_clipped,control_position1,control_position2,control_position_time1,control_position_time2,application_delivery_time1,application_delivery_time2\n";
    const Tick finish = static_cast<Tick>(std::llround(duration / dt));
    for (Tick k = 0; k <= finish; ++k) {
      const double now = k * dt;
      // All three states are already at now. receive() cannot advance an axis ahead.
      while (!commands.empty() && commands.front().due <= k) {
        const auto cmd = commands.front(); commands.pop_front();
        if (cmd.due != k || cmd.sent + command_delay != k || cmd.control > cmd.sent) {
          throw std::runtime_error("Noncausal command");
        }
        for (auto source : cmd.sources) {
          if (source > cmd.control) {throw std::runtime_error("Controller used future sample");}
        }
        const auto replies = motors[cmd.axis].receive(cmd.frame, now);
        if (replies.size() != 1) {throw std::runtime_error("Missing MIT response");}
        applied[cmd.axis] = dm_motor::decode_mit(cmd.frame, pars[cmd.axis].motor.mapping);
        applied_friction[cmd.axis] = cmd.friction;
        applied_coupling[cmd.axis] = cmd.coupling;
        applied_at[cmd.axis] = k;
        const Tick nominal_received = k + feedback_delay;
        Tick received = nominal_received;
        if (int(cmd.axis + 1) == batch_axis) {
          if (nominal_received == batch_at - batch_window) {received = batch_at - batch_gap;}
          else if (nominal_received > batch_at - batch_window && nominal_received <= batch_at) {
            received = batch_at;
          }
        }
        const Tick delivery = int(cmd.axis + 1) == pause_axis && received >= pause_start &&
          received < pause_start + pause_duration ? pause_start + pause_duration : received;
        const PendingFeedback pending{delivery, k, received, nominal_received,
          cmd.axis, cmd.sequence, replies.front()};
        // A paused application subscription must not block B/C. Keep every generated
        // frame and its original transport timestamp; drain delayed A frames in order.
        const auto receive_place = std::upper_bound(transport_feedbacks.begin(), transport_feedbacks.end(), received,
          [](Tick time, const PendingFeedback & item) {return time < item.received;});
        transport_feedbacks.insert(receive_place, pending);
        events << cmd.axis + 1 << ',' << cmd.sequence << ',' << cmd.control << ',' << cmd.sent
               << ',' << k << ',' << received << ',' << cmd.sources[0] << ','
               << cmd.sources[1] << ',' << bytes(cmd.frame) << ',' << bytes(replies.front())
               << ',' << delivery << ',' << nominal_received << '\n';
      }
      while (!transport_feedbacks.empty() && transport_feedbacks.front().received <= k) {
        auto f = transport_feedbacks.front(); transport_feedbacks.pop_front();
        if (f.received != k) {throw std::runtime_error("Late simulated transport delivery");}
        // Production Bus decodes/unwraps at transport receipt, before any application pause.
        f.state = feedback_runtime.receive(f.axis, f.frame, f.received * dt);
        if (f.sequence != received_sequences[f.axis] + 1 ||
          (received_counts[f.axis] && f.state.feedback_count != received_counts[f.axis] + 1))
        {throw std::runtime_error("Receive collection lost, duplicated or reordered a frame");}
        received_sequences[f.axis] = f.sequence;
        received_counts[f.axis] = f.state.feedback_count;
        receptions << f.axis + 1 << ',' << f.sequence << ',' << f.sampled << ','
                   << f.nominal_received << ',' << f.received << ',' << f.due << ','
                   << f.state.feedback_count << ',' << bytes(f.frame) << '\n';
        const auto place = std::upper_bound(feedbacks.begin(), feedbacks.end(), f.due,
          [](Tick time, const PendingFeedback & item) {return time < item.due;});
        feedbacks.insert(place, f);
      }
      while (!feedbacks.empty() && feedbacks.front().due <= k) {
        const auto f = feedbacks.front(); feedbacks.pop_front();
        const auto decoded = dm_motor::decode_feedback(f.frame, f.axis + 1, 0,
          pars[f.axis].motor.mapping);
        if (!decoded || f.due != k || f.sampled + feedback_delay != f.nominal_received ||
          f.nominal_received > f.received || f.received > k || decoded->status != 1) {
          throw std::runtime_error("Invalid or noncausal motor feedback");
        }
        // This is the only assignment of controller-visible measurements.
        latest[f.axis] = {f.received * dt, f.state.absolute_position,
          f.state.feedback.v, f.state.feedback.t, f.sequence};
        native_position[f.axis] = decoded->p;
        position_wraps[f.axis] = f.state.position_wraps;
        delivered_at[f.axis] = k;
        if (f.axis < 2) {
          pairs.push(f.axis, latest[f.axis]);
          auto & history = decoded_history[f.axis];
          history.push_back(latest[f.axis]);
          if (history.size() > 16) {history.pop_front();}
        }
        sampled[f.axis] = f.sampled;
        if (!seen[f.axis] && f.axis < 2) {origins[f.axis] = decoded->p;}
        seen[f.axis] = true;
      }
      if (k >= controller_phase && (k - controller_phase) % period == 0) {
        // Production ENABLING waits for a bracket before entering ACTIVE. A
        // shifted controller phase can see both axes before their histories overlap.
        const bool synchronization_ready = position_mode != "synchronized" ||
          (seen[0] && seen[1] && pairs.common_time_positions(now,
          number("synchronization_max_age", .02), number("synchronization_max_bracket_gap", .005)));
        if (seen[0] && seen[1] && !synchronization_started && !synchronization_ready && now >= .020) {
          throw std::runtime_error("No common-time positions during startup: stop required");
        }
        if (seen[0] && seen[1] && (synchronization_started || synchronization_ready)) {
          for (size_t i = 0; i < 2; ++i) {
            const double midpoint = std::clamp(latest[i].velocity + limits[i].mapping.v / 4095.,
              -limits[i].mapping.v, limits[i].mapping.v);
            filtered[i] = filters[i].update(midpoint, latest[i].time);
            friction[i] = compensators[i].step(midpoint, .002,
              settings.friction_torque_limit, fraction, settings.extra_viscous[i],
              settings.extra_viscous_velocity_limit);
          }
          std::array<Sample, 2> control_samples{{latest[0], latest[1]}};
          std::optional<CommonTimePositions> synchronized;
          if (position_mode == "synchronized") {
            std::string rejection_reason;
            synchronized = pairs.common_time_positions(now,
              number("synchronization_max_age", .02), number("synchronization_max_bracket_gap", .005),
              &rejection_reason);
            controls << now << ',' << bool(synchronized) << ','
                     << (synchronized ? synchronized->time : -1) << ','
                     << latest[0].time << ',' << latest[1].time;
            for (size_t i = 0; i < 2; ++i) {controls << ',' << (synchronized ? synchronized->position[i] : 0);}
            for (size_t i = 0; i < 2; ++i) {controls << ',' << (synchronized ? synchronized->bracket_gap[i] : 0);}
            for (size_t i = 0; i < 2; ++i) {controls << ',' << (synchronized ? synchronized->latest_age[i] : 0);}
            for (size_t i = 0; i < 2; ++i) {
              double lower_age = -1, upper_age = -1;
              if (synchronized) {
                const auto & history = decoded_history[i];
                const auto upper = std::lower_bound(history.begin(), history.end(), synchronized->time,
                  [](const Sample & sample, double time) {return sample.time < time;});
                if (upper != history.end()) {
                  upper_age = now - upper->time;
                  if (upper->time == synchronized->time) {lower_age = upper_age;}
                  else if (upper != history.begin()) {lower_age = now - std::prev(upper)->time;}
                }
              }
              controls << ',' << lower_age << ',' << upper_age;
            }
            controls << ',' << rejection_reason << '\n';
            if (!synchronized) {
              // The corresponding production action is STOP, never latest fallback.
              throw std::runtime_error("No common-time positions: stop required at " +
                std::to_string(now) + ": " + rejection_reason);
            }
            synchronization_started = true;
          }
          const double common_time = std::min(latest[0].time, latest[1].time);
          for (size_t i = 0; i < 2; ++i) {
            control_position_time[i] = latest[i].time;
            if (position_mode == "common_time" && latest[i].time > common_time) {
              const auto & history = decoded_history[i];
              auto right = std::lower_bound(history.begin(), history.end(), common_time,
                [](const Sample & sample, double time) {return sample.time < time;});
              if (right == history.begin() || right == history.end()) {
                throw std::runtime_error("Common-time position lacks delivered decoded bracket");
              }
              const auto & left = *std::prev(right);
              const double weight = (common_time - left.time) / (right->time - left.time);
              control_samples[i].position = left.position + weight * (right->position - left.position);
              control_position_time[i] = common_time;
            }
            control_position[i] = control_samples[i].position;
            if (synchronized) {
              control_position[i] = synchronized->position[i];
              control_position_time[i] = synchronized->time;
            }
          }
          // Position-only diagnostic: velocity, torque and their timestamps remain latest.
          // Both interpolation endpoints were already delivered; no physical truth enters.
          if (synchronized) {
            // Same production path as controller.hpp, supplying only the new
            // optional spring positions. All velocity/damping inputs stay latest.
            auto reference_samples = control_samples;
            for (size_t i = 0; i < 2; ++i) {reference_samples[i].velocity = filtered[i];}
            auto desired = shared_reference_targets(reference_samples, origins, settings);
            for (size_t i = 0; i < 2; ++i) {desired[i].t = friction[i];}
            result.output = symmetric_targets(desired, control_samples, limits, settings, 1, {}, synchronized);
            result.coupling = result.output.coupling_torque;
            result.headroom_clipped = result.output.coupling_scale + 1e-14 < result.output.gain_scale;
          } else {
            result = bench_control::compute({control_samples, origins, filtered,
              friction, limits, settings, 1, 0});
          }
          for (size_t i = 0; i < 2; ++i) {
            targets[i] = result.output.targets[i];
            target_friction[i] = result.output.friction_torque[i];
            target_coupling[i] = result.coupling[i];
            target_control[i] = k;
            target_sources[i] = {sampled[0], sampled[1]};
          }
        }
        const auto ref = reference(now);
        // C alone owns the prescribed trajectory. A and B have no environmental target.
        targets[2] = {pars[2].physics.initial_position + ref.first, ref.second,
          touching ? 0. : c_kp, touching ? 0. : c_kd, 0};
        target_control[2] = k;
      }
      for (size_t i = 0; i < 3; ++i) {
        const Tick phase = i == 1 ? b_phase : 0;
        if (k >= phase && (k - phase) % period == 0) {
          const auto frame = i < 2 ? bench_control::wire(i + 1, targets[i], limits[i]) :
            dm_motor::mit_frame(i + 1, targets[i], pars[i].motor.mapping);
          commands.push_back({k + command_delay, k, target_control[i], i, ++sequences[i],
            target_sources[i], frame, target_friction[i], target_coupling[i]});
        }
      }
      // Snapshot both ends before applying either torque; opposite encoders need same signs.
      // Disconnected C cannot influence A/B. It may keep moving independently in release_pair.
      const bool coupled = scenario != "touch_pair" &&
        !(scenario == "release_pair" && now >= release_time);
      const auto force = coupled ? link.evaluate(motors[0].state().position, motors[0].state().velocity,
        motors[2].state().position, motors[2].state().velocity) : three_motor_rig::Coupling::Result{};
      const double probe = touching && now >= probe_start &&
        now < probe_start + probe_duration ? probe_torque : 0;
      const double probe1 = scenario == "touch_pair" ? probe : 0;
      const double probe3 = scenario == "touch" ? probe : 0;
      const std::array<double, 3> external{{force.torque1 + probe1, 0, force.torque3 + probe3}};
      for (size_t i = 0; i < 3; ++i) {motors[i].external_torque(external[i], 2 * dt, now);}
      if (k % trace_period == 0) {
        const auto ref = reference(now);
        trace << now << ',' << pars[2].physics.initial_position + ref.first << ',' << ref.second
              << ',' << probe1 << ',' << probe3 << ',' << coupled
              << ',' << motors[0].state().position + motors[2].state().position - link.offset
              << ',' << force.extension << ',' << force.torque1 << ',' << force.torque3
              << ',' << force.energy << ',' << force.dissipation;
        for (size_t i = 0; i < 3; ++i) {
          const auto & s = motors[i].state();
          trace << ',' << s.position << ',' << s.velocity << ',' << s.motor_torque
                << ',' << latest[i].position << ',' << latest[i].velocity << ',' << latest[i].torque
                << ',' << sampled[i] * dt << ',' << applied_at[i] * dt
                << ',' << applied[i].p << ',' << applied[i].v << ',' << applied[i].kp
                << ',' << applied[i].kd << ',' << applied[i].t << ',' << s.friction_torque
                << ',' << external[i] << ',' << int(motors[i].status())
                << ',' << native_position[i] << ',' << position_wraps[i]
                << ',' << s.mos_temperature << ',' << s.rotor_temperature;
        }
        trace << ',' << filtered[0] << ',' << filtered[1] << ',' << friction[0] << ',' << friction[1]
              << ',' << applied_friction[0] << ',' << applied_friction[1]
              << ',' << applied_coupling[0] << ',' << applied_coupling[1]
              << ',' << motors[0].state().position - motors[1].state().position
              << ',' << result.headroom_clipped
              << ',' << control_position[0] << ',' << control_position[1]
              << ',' << control_position_time[0] << ',' << control_position_time[1]
              << ',' << delivered_at[0] * dt << ',' << delivered_at[1] * dt << '\n';
      }
      if (k != finish) {
        // No force is recomputed between these advances: all axes use the same old snapshot.
        for (auto & motor : motors) {motor.advance((k + 1) * dt);}
      }
    }
    std::cout << "Completed offline three-motor case, speed=" << speed
              << ", fraction=" << fraction << ", physics_steps=" << finish
              << ", coupling_mode=" << coupling_mode
              << ", motor3_model=" << (independent_c ? "explicit_motor_3" : "assumed_copy_of_motor_1")
              << ", friction_torque_limit=" << settings.friction_torque_limit
              << ", C_kp=" << c_kp << ", C_kd=" << c_kd
              << ", C_V_BW_register=" << c_bandwidth << " (no implicit dynamics)"
              << ", position_feedback_mode=" << position_mode << '\n';
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
