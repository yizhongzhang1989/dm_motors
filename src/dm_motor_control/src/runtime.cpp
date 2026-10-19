#include "dm_motor_control/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace dm_motor
{
std::string motor_status_text(uint8_t status)
{
  switch (status) {
    case 0: return "disabled";
    case 1: return "enabled";
    case 3: return "output encoder calibration fault";
    case 4: return "sensor output fault";
    case 5: return "motor encoder calibration fault";
    case 8: return "overvoltage";
    case 9: return "undervoltage";
    case 10: return "overcurrent";
    case 11: return "MOS overtemperature";
    case 12: return "motor winding overtemperature";
    case 13: return "communication loss";
    case 14: return "overload";
    default: return "unknown device status " + std::to_string(status);
  }
}
namespace
{
uint32_t bits(float value)
{
  uint32_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}
double number(uint32_t raw)
{
  float value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}
uint32_t raw_value(const Frame & f)
{
  uint32_t raw = 0;
  for (unsigned i = 0; i < 4; ++i) {raw |= uint32_t(f.data[4 + i]) << (8 * i);}
  return raw;
}
void put_raw(Frame & f, unsigned offset, uint32_t raw)
{
  for (unsigned i = 0; i < 4; ++i) {f.data[offset + i] = (raw >> (8 * i)) & 255;}
}
void bounded(double value, double low, double high, const char * field)
{
  if (!std::isfinite(value) || value < low || value > high) {
    throw std::invalid_argument(std::string(field) + " outside configured range");
  }
}
const RegisterInfo & info(uint8_t rid)
{
  const auto & catalog = register_catalog();
  auto it = std::find_if(catalog.begin(), catalog.end(),
    [rid](const RegisterInfo & r) {return r.rid == rid;});
  if (it == catalog.end()) {throw std::invalid_argument("unknown register");}
  return *it;
}
void validate_raw(uint8_t rid, uint32_t raw)
{
  const auto & r = info(rid);
  if (!r.writable) {throw std::invalid_argument("read-only register");}
  const double value = r.integer ? double(raw) : number(raw);
  if (!std::isfinite(value) ||
    (r.minimum && (value < *r.minimum || (!r.minimum_inclusive && value == *r.minimum))) ||
    (r.maximum && (value > *r.maximum || (!r.maximum_inclusive && value == *r.maximum))))
  {
    throw std::invalid_argument("register value outside published range");
  }
}
Frame parameter(uint8_t id, uint8_t opcode, uint8_t rid, uint32_t raw = 0)
{
  // Explicit SDK-compatible eight-byte padding for legacy USB2CAN.
  Frame f{0x7ff, {id, 0, opcode, rid, 0, 0, 0, 0}};
  put_raw(f, 4, raw);
  return f;
}
bool parameter_packet(const Frame & f)
{
  return (f.length == 4 || f.length == 8) && f.data[0] >= 1 && f.data[0] <= 15 &&
         f.data[1] == 0 &&
         (f.data[2] == 0x33 || f.data[2] == 0x55 || f.data[2] == 0xaa);
}
}

const std::vector<RegisterInfo> & register_catalog()
{
  const double fm = std::numeric_limits<float>::max();
  static const std::vector<RegisterInfo> catalog{
    {0, "UV_Value", false, true, "V", 10, fm, false, true},
    {1, "KT_Value", false, true, "Nm/A", 0, fm},
    {2, "OT_Value", false, true, "degC", 80, 200, true, false},
    {3, "OC_Value", false, true, "ratio", 0, 1, false, false},
    {4, "ACC", false, true, "raw (rotor acceleration; verify firmware scaling)", 0, fm, false, false},
    {5, "DEC", false, true, "raw (rotor acceleration; verify firmware scaling)", -fm, 0, true, false},
    {6, "MAX_SPD", false, true, "rotor rad/s", 0, fm, false, true},
    {7, "MST_ID", true, true, "", 0, 2047},
    {8, "ESC_ID", true, true, "", 0, 2047},
    {9, "TIMEOUT", true, true, "50 us", 0, 4294967295.0},
    {10, "CTRL_MODE", true, true, "", 0, 4},
    {11, "Damp", false, false, "", {}, {}},
    {12, "Inertia", false, false, "", {}, {}},
    {13, "hw_ver", true, false, "", {}, {}},
    {14, "sw_ver", true, false, "", {}, {}},
    {15, "SN", true, false, "", {}, {}},
    {16, "NPP", true, false, "", {}, {}},
    {17, "Rs", false, false, "ohm", {}, {}},
    {18, "Ls", false, false, "H", {}, {}},
    {19, "Flux", false, false, "", {}, {}},
    {20, "Gr", false, false, "ratio", {}, {}},
    {21, "PMAX", false, true, "rad", 0, fm, false, true},
    {22, "VMAX", false, true, "rad/s", 0, fm, false, true},
    {23, "TMAX", false, true, "Nm", 0, fm, false, true},
    {24, "I_BW", false, true, "", 100, 1e4},
    {25, "KP_ASR", false, true, "", 0, fm},
    {26, "KI_ASR", false, true, "", 0, fm},
    {27, "KP_APR", false, true, "", 0, fm},
    {28, "KI_APR", false, true, "", 0, fm},
    {29, "OV_Value", false, true, "V", {}, {}},
    {30, "GREF", false, true, "ratio", 0, 1, false, true},
    {31, "Deta", false, true, "", 1, 30},
    {32, "V_BW", false, true, "", 0, 500, false, false},
    {33, "IQ_c1", false, true, "", 100, 1e4},
    {34, "VL_c1", false, true, "", 0, 1e4, false, true},
    {35, "can_br", true, true, "code", 0, 9},
    {36, "sub_ver", true, false, "", {}, {}},
    {37, "Boot_ver", true, false, "", {}, {}},
    {55, "dir", false, false, "", {}, {}},
    {56, "m_off", false, false, "rad", {}, {}},
    {59, "Imax", false, false, "A", {}, {}},
    {60, "VBus", false, false, "V", {}, {}},
    {61, "Tpcb", false, false, "degC", {}, {}},
    {62, "Tmtr", false, false, "degC", {}, {}},
    {63, "I_U_OFF", false, false, "", {}, {}},
    {64, "I_V_OFF", false, false, "", {}, {}},
    {65, "I_W_OFF", false, false, "", {}, {}},
    {80, "p_m", false, false, "rad", {}, {}},
    {81, "xout", false, false, "rad", {}, {}}
  };
  return catalog;
}

uint16_t control_id(uint8_t id, Mode mode)
{
  if (id < 1 || id > 15 || unsigned(mode) < 1 || unsigned(mode) > 4) {
    throw std::invalid_argument("invalid motor ID or mode");
  }
  return id + (unsigned(mode) - 1) * 0x100;
}
double position_command_limit(Mode mode, const Mapping & mapping, const Limits & limits)
{
  const auto resolved = resolve_limits(mapping, limits);
  return mode == Mode::Mit || limits.p != 0 ? resolved.p : std::numeric_limits<float>::max();
}
Frame target_frame(uint8_t id, const Target & t, const Mapping & mapping, const Limits & limits)
{
  const auto l = resolve_limits(mapping, limits);
  const double fields[] = {t.position, t.velocity, t.kp, t.kd, t.torque,
    t.velocity_limit, t.current_ratio};
  for (double value : fields) {
    if (!std::isfinite(value)) {throw std::invalid_argument("non-finite target");}
  }
  if (t.mode == Mode::Mit) {
    if (t.velocity_limit != 0 || t.current_ratio != 0) {
      throw std::invalid_argument("non-MIT fields supplied to MIT mode");
    }
    Command c{t.position, t.velocity, t.kp, t.kd, t.torque};
    validate_command(c, mapping, limits);
    return mit_frame(id, c, mapping);
  }
  Frame f;
  f.id = control_id(id, t.mode);
  if (t.kp != 0 || t.kd != 0 || t.torque != 0) {
    throw std::invalid_argument("MIT-only fields supplied to another mode");
  }
  if (t.mode == Mode::Velocity) {
    if (t.position != 0 || t.velocity_limit != 0 || t.current_ratio != 0) {
      throw std::invalid_argument("unused velocity-mode fields must be zero");
    }
    bounded(t.velocity, -l.v, l.v, "velocity");
    f.length = 4;
    put_raw(f, 0, bits(static_cast<float>(t.velocity)));
  } else {
    const double position_limit = position_command_limit(t.mode, mapping, limits);
    bounded(t.position, -position_limit, position_limit, "position");
    put_raw(f, 0, bits(static_cast<float>(t.position)));
    if (t.mode == Mode::PositionVelocity) {
      if (t.velocity != 0 || t.current_ratio != 0) {
        throw std::invalid_argument("use velocity_limit in position-velocity mode");
      }
      bounded(t.velocity_limit, 0, l.v, "velocity_limit");
      put_raw(f, 4, bits(static_cast<float>(t.velocity_limit)));
    } else {
      if (t.velocity != 0) {throw std::invalid_argument("use velocity_limit in position-force mode");}
      bounded(t.velocity_limit, 0, std::min(100.0, l.v), "velocity_limit");
      bounded(t.current_ratio, 0, 1, "current_ratio");
      const auto v = static_cast<uint16_t>(std::floor(t.velocity_limit * 100));
      const auto i = static_cast<uint16_t>(std::floor(t.current_ratio * 10000));
      f.data[4] = v & 255; f.data[5] = v >> 8;
      f.data[6] = i & 255; f.data[7] = i >> 8;
    }
  }
  return f;
}

struct Bus::Impl
{
  struct Motor
  {
    MotorSnapshot state;
    Target target;
    double last_feedback{-1}, last_command{-1}, transition{0}, last_stop{-1};
    unsigned startup{0};
    bool startup_status_requested{false};
    bool anchor_pending{false};
    double anchor_time{-1}, tracking_time{-1};
    double anchor_idle_since{-1};
  };
  enum class Kind {Read, Write, ModeWrite, Save, Clear, Zero, StartupRead, StartupWatchdog};
  struct Transaction
  {
    uint64_t op{0};
    size_t motor{0};
    Kind kind{Kind::Read};
    uint8_t rid{0};
    uint32_t raw{0};
    unsigned stage{0};
    double sent{0}, deadline{0};
    uint64_t feedback_before{0};
    double preparation_started{-1};
    double reservation_wait_started{-1};
  };
  std::unique_ptr<Transport> transport;
  std::vector<MotorConfig> configs;
  std::vector<Motor> motors;
  std::function<void(const std::string &, const MotorSnapshot &, double)> feedback_callback;
  std::deque<Transaction> queue;
  std::map<uint64_t, RegisterOperation> history;
  uint64_t next{1};
  std::string init_error;
  bool initialized{false}, failed{false};
  double last_tick{-1}, quiet_until{0};
  size_t startup_motor{0};

  size_t index(const std::string & name) const
  {
    for (size_t i = 0; i < configs.size(); ++i) {
      if (configs[i].name == name) {return i;}
    }
    throw std::invalid_argument("unknown motor: " + name);
  }
  bool fresh(size_t i, double now) const
  {
    return motors[i].last_feedback >= 0 && now >= motors[i].last_feedback &&
           now - motors[i].last_feedback <= configs[i].feedback_timeout;
  }
  bool maintenance(double now, bool clear = false, size_t selected = 0) const
  {
    if (!initialized || failed || !std::isfinite(now) || now < last_tick) {return false;}
    for (size_t i = 0; i < motors.size(); ++i) {
      const auto & m = motors[i];
      if (!fresh(i, now)) {return false;}
      if (clear && selected == i && m.state.phase == Phase::Fault) {
        if (m.state.feedback.status == 1) {return false;}
      } else if (m.state.phase != Phase::Disabled || m.state.feedback.status != 0) {
        return false;
      }
    }
    return true;
  }
  void stop(size_t i)
  {
    transport->send(special_frame(control_id(configs[i].id, motors[i].state.mode), 0xfd));
  }
  void fault(size_t i, const std::string & reason)
  {
    auto & m = motors[i];
    if (!m.state.host_fault) {m.state.error = reason; m.last_stop = -1;}
    m.state.host_fault = true;
    m.state.phase = Phase::Fault;
    m.target = {};
  }
  void io_failure(const std::string & reason)
  {
    failed = true;
    if (!initialized) {init_error = reason;}
    for (size_t i = 0; i < motors.size(); ++i) {
      motors[i].state.absolute_position_valid = false;
      motors[i].anchor_pending = false;
      fault(i, reason);
      // A failed mode write may have changed the device despite a lost response.
      for (unsigned mode = 1; mode <= 4; ++mode) {
        try {transport->send(special_frame(control_id(configs[i].id, Mode(mode)), 0xfd));}
        catch (...) {}
      }
    }
    for (auto & item : history) {
      if (item.second.state == OperationState::Pending) {
        item.second.state = OperationState::Unknown;
        item.second.message = reason;
      }
    }
    queue.clear();
  }
  uint64_t enqueue(size_t i, Kind kind, uint8_t rid, uint32_t raw, double now,
    const std::string & rejection = {})
  {
    if (history.size() >= 256) {
      auto it = std::find_if(history.begin(), history.end(), [](const auto & entry) {
        return entry.second.state != OperationState::Pending;
      });
      if (it != history.end()) {history.erase(it);}
    }
    const uint64_t id = next++;
    std::string error = rejection;
    if (queue.size() >= 64) {error = "pending operation queue full";}
    if (error.empty() && !maintenance(now, kind == Kind::Clear, i)) {
      error = "maintenance requires fresh confirmed disabled status on the entire bus; selected phase=" +
        std::to_string(static_cast<unsigned>(motors[i].state.phase)) + ", status=" +
        std::to_string(motors[i].state.feedback.status) + ", age=" +
        std::to_string(now - motors[i].last_feedback) + ", initialized=" +
        std::to_string(initialized) + ", failed=" + std::to_string(failed);
    }
    if (error.empty() && transport->maintenance_blocked()) {
      error = "CAN maintenance requires all registered clients quiet";
    }
    history.emplace(id, RegisterOperation{id,
      error.empty() ? OperationState::Pending : OperationState::Rejected,
      configs[i].name, rid, raw, error});
    if (error.empty()) {
      queue.push_back({id, i, kind, rid, raw});
      motors[i].target = {};
    }
    return id;
  }
  void finish(OperationState state, const std::string & message, uint32_t value, double now)
  {
    const auto tx = queue.front();
    queue.pop_front();
    quiet_until = now + 0.01;
    if (tx.op) {
      auto & op = history.at(tx.op);
      op.state = state; op.message = message; op.raw_value = value;
    }
    if (state != OperationState::Confirmed) {
      if (!tx.op || state == OperationState::Unknown ||
        ((tx.kind == Kind::ModeWrite || tx.kind == Kind::StartupWatchdog) && tx.stage > 0)) {
        io_failure(message);
      }
      return;
    }
    auto & m = motors[tx.motor];
    auto & c = configs[tx.motor];
    if (tx.kind == Kind::ModeWrite) {
      m.state.mode = Mode(value); c.mode = Mode(value); m.target = {};
    }
    if (tx.kind == Kind::StartupRead) {
      if (tx.rid == 7 && value != c.master) {
        io_failure("startup MST_ID mismatch"); return;
      }
      if (tx.rid == 10 && value != unsigned(c.mode)) {
        if (!c.configure_mode_on_startup || value < 1 || value > 4) {
          io_failure("startup CTRL_MODE mismatch; configure desired mode while disabled before retry");
          return;
        }
        m.state.mode = Mode(value);
      }
      if (tx.rid == 21) {m.state.mapping.p = number(value);}
      if (tx.rid == 22) {m.state.mapping.v = number(value);}
      if (tx.rid == 23) {
        m.state.mapping.t = number(value);
        try {
          validate_mapping(m.state.mapping);
          m.state.limits = resolve_limits(m.state.mapping, c.limits);
          c.mapping = m.state.mapping;
        } catch (const std::exception & e) {io_failure(e.what()); return;}
      }
      if (tx.rid == 80) {
        const double position = number(value);
        if (!std::isfinite(position)) {
          io_failure("startup p_m position is not finite"); return;
        }
        m.state.absolute_position = position;
        m.anchor_pending = true;
        m.anchor_time = now;
        m.transition = now;
      }
      ++m.startup;
    } else if (tx.kind == Kind::StartupWatchdog) {
      ++m.startup;
    }
    // A separate status window between transactions refreshes maintenance preconditions.
    for (const auto & config : configs) {transport->send(query_frame(config.id));}
  }
  void startup(double now)
  {
    if (initialized || failed || !queue.empty() || now < quiet_until ||
      !transport->maintenance_available()) {return;}
    if (startup_motor == motors.size()) {initialized = true; return;}
    auto & m = motors[startup_motor];
    static constexpr uint8_t rids[] = {7, 10, 21, 22, 23};
    if (m.startup < 5) {
      queue.push_back({0, startup_motor, Kind::StartupRead, rids[m.startup]});
    } else if (m.startup == 5) {
      if (!m.startup_status_requested) {
        m.startup_status_requested = true;
        m.transition = now;
        transport->send(query_frame(configs[startup_motor].id));
        return;
      }
      if (m.last_feedback < 0 || m.last_feedback <= m.transition) {
        if (now - m.transition > 0.5) {io_failure("startup disabled feedback timeout");}
      } else if (m.state.feedback.status != 0) {
        io_failure("startup motor is not disabled");
      } else {
        m.startup = 6;
      }
    } else if (m.startup == 6) {
      if (m.state.mode != configs[startup_motor].mode) {
        queue.push_back({0, startup_motor, Kind::ModeWrite, 10, unsigned(configs[startup_motor].mode)});
      } else {
        const auto raw = static_cast<uint32_t>(std::llround(configs[startup_motor].watchdog / 50e-6));
        queue.push_back({0, startup_motor, Kind::StartupWatchdog, 9, raw});
      }
    } else if (m.startup == 7) {
      // Finish bus-wide commissioning before anchoring: another node's startup
      // parameter burst suppresses our feedback and can make a new anchor ambiguous.
      if (!transport->activity_available() || !fresh(startup_motor, now) ||
        m.state.feedback.status != 0)
      {
        m.anchor_idle_since = -1;
        return;
      }
      if (m.anchor_idle_since < 0) {m.anchor_idle_since = now;}
      if (now - m.anchor_idle_since < configs[startup_motor].feedback_timeout) {return;}
      queue.push_back({0, startup_motor, Kind::StartupRead, 80});
    } else if (!m.state.absolute_position_valid) {
      if (now - m.transition > 0.5) {io_failure("startup absolute position anchor timeout");}
    } else if (!transport->activity_available()) {
      return;
    } else {
      ++startup_motor;
    }
  }
  void transaction(double now)
  {
    if (queue.empty() || failed) {return;}
    auto & tx = queue.front();
    auto & m = motors[tx.motor];
    const auto & c = configs[tx.motor];
    if (tx.stage && now > tx.deadline) {
      finish(OperationState::Unknown,
        "operation timed out; outcome unknown (no automatic retry); register=" +
        std::to_string(tx.rid) + ", stage=" + std::to_string(tx.stage),
        tx.raw, now);
      return;
    }
    if (tx.stage == 0) {
      if (now < quiet_until) {return;}
      if (!transport->prepare_maintenance()) {
        if (tx.reservation_wait_started < 0) {tx.reservation_wait_started = now;}
        if (tx.op && transport->maintenance_blocked()) {
          finish(OperationState::Rejected,
            "CAN maintenance unavailable: another client is active",
            tx.raw, now);
        } else if (tx.op && now - tx.reservation_wait_started > 2.0) {
          finish(OperationState::Rejected,
            "CAN maintenance reservation timed out before any device request was sent", tx.raw, now);
        }
        return;
      }
      // Revalidate at dispatch, not only when the service request was queued.
      const bool startup_mode = !tx.op && tx.kind == Kind::ModeWrite;
      const bool startup_disabled = m.state.phase == Phase::Disabled &&
        m.state.feedback.status == 0 && fresh(tx.motor, now);
      if ((tx.op && !maintenance(now, tx.kind == Kind::Clear, tx.motor)) ||
        (startup_mode && !startup_disabled))
      {
        const bool quiet_phase = m.state.phase == Phase::Disabled ||
          (tx.kind == Kind::Clear && m.state.phase == Phase::Fault);
        const bool quiet_status = m.state.feedback.status == 0 ||
          (tx.kind == Kind::Clear && m.state.feedback.status != 1);
        if (quiet_phase && quiet_status && !fresh(tx.motor, now)) {
          if (tx.preparation_started < 0) {tx.preparation_started = now;}
          if (now - tx.preparation_started < 0.5) {
            transport->send(query_frame(c.id));
            return;
          }
        }
        finish(OperationState::Rejected, "disabled maintenance precondition changed", tx.raw, now);
        return;
      }
      tx.deadline = now + 0.5;
      tx.sent = now;
      tx.feedback_before = m.state.feedback_count;
      if (tx.kind == Kind::Read || tx.kind == Kind::StartupRead) {
        transport->send(parameter(c.id, 0x33, tx.rid)); tx.stage = 2;
      } else if (tx.kind == Kind::Write || tx.kind == Kind::ModeWrite ||
        tx.kind == Kind::StartupWatchdog)
      {
        transport->send(parameter(c.id, 0x55, tx.rid, tx.raw)); tx.stage = 1;
      } else if (tx.kind == Kind::Save) {
        transport->send(parameter(c.id, 0xaa, 1)); tx.stage = 2;
      } else {
        transport->send(special_frame(control_id(c.id, m.state.mode),
          tx.kind == Kind::Clear ? 0xfb : 0xfe));
        tx.stage = 1;
      }
    } else if (tx.stage == 1 && now - tx.sent >= 0.01) {
      // Drain write echoes before issuing an independent readback / fresh status query.
      if (tx.kind == Kind::Write || tx.kind == Kind::ModeWrite ||
        tx.kind == Kind::StartupWatchdog)
      {
        transport->send(parameter(c.id, 0x33, tx.rid));
      } else {
        tx.feedback_before = m.state.feedback_count;
        transport->send(query_frame(c.id));
      }
      tx.stage = 2;
    }
  }
  void receive(const Frame & f, double now)
  {
    if (f.channel != 0 || f.id > 0x7ff || (f.brs && !f.fd)) {return;}
    if (parameter_packet(f)) {
      if (queue.empty()) {return;}
      const auto tx = queue.front();
      const auto & c = configs[tx.motor];
      if (tx.stage != 2 || f.id != c.master || f.data[0] != c.id ||
        f.data[3] != (tx.kind == Kind::Save ? 1 : tx.rid)) {return;}
      if (tx.kind == Kind::Save) {
        if (f.data[2] == 0xaa && now - tx.sent >= 0.03) {
          finish(OperationState::Confirmed,
            "save acknowledged; persistence requires controlled power-cycle verification", 0, now);
        } else if (f.data[2] == 0xaa) {
          // Store acknowledgement, but observe the published Flash programming budget.
          queue.front().stage = 3;
        }
      } else if (f.length == 8 && f.data[2] == 0x33) {
        const uint32_t value = raw_value(f);
        const bool write = tx.kind == Kind::Write || tx.kind == Kind::ModeWrite ||
          tx.kind == Kind::StartupWatchdog;
        if (tx.kind == Kind::ModeWrite && value == tx.raw) {
          // A mode operation is terminal only after post-change disabled status arrives.
          // This makes CONFIRMED immediately usable by an explicit enable request.
          auto & m = motors[tx.motor];
          m.state.mode = Mode(value);
          configs[tx.motor].mode = Mode(value);
          queue.front().stage = 3;
          queue.front().feedback_before = m.state.feedback_count;
          transport->send(query_frame(c.id));
          return;
        }
        finish(write && value != tx.raw ? OperationState::Rejected : OperationState::Confirmed,
          write && value != tx.raw ? "independent readback differs from requested value" :
          "fresh register readback", value, now);
      }
      return;
    }
    for (size_t i = 0; i < motors.size(); ++i) {
      auto fb = decode_feedback(f, configs[i].id, configs[i].master, motors[i].state.mapping);
      if (!fb) {continue;}
      auto & m = motors[i];
      const double received = f.received_steady ? f.received_steady * 1e-9 : now;
      const bool zero_confirmation = !queue.empty() && queue.front().motor == i &&
        queue.front().kind == Kind::Zero && queue.front().stage == 2 &&
        m.state.feedback_count >= queue.front().feedback_before &&
        fb->status == 0 && std::abs(fb->p) <= m.state.mapping.p * 2 / 65535 + 1e-6;
      if (zero_confirmation) {
        m.state.absolute_position = 0;
        m.state.absolute_position_valid = true;
        m.state.position_wraps = 0;
        m.anchor_pending = false;
        m.tracking_time = received;
      } else if (m.anchor_pending && received > m.anchor_time) {
        const double near_zero_speed = std::max(0.1, m.state.mapping.v * 2 / 4095);
        if (fb->status != 0 || std::abs(fb->v) > near_zero_speed) {
          io_failure("startup absolute position requires stationary disabled feedback"); return;
        }
        const double period = 2 * m.state.mapping.p * 65536 / 65535;
        if (std::abs(std::remainder(m.state.absolute_position - fb->p, period)) >
          0.01 + near_zero_speed * (received - m.anchor_time))
        {
          io_failure("startup p_m differs from compact position anchor"); return;
        }
        m.state.absolute_position_valid = true;
        m.state.position_wraps = 0;
        m.anchor_pending = false;
        m.tracking_time = received;
      } else if (m.state.absolute_position_valid) {
        // One full 16-bit rollover includes 65536 steps, not 65535.
        const double period = 2 * m.state.mapping.p * 65536 / 65535;
        const double elapsed = received - m.tracking_time;
        if (elapsed < 0 || elapsed * m.state.mapping.v >= period / 2) {
          m.state.absolute_position_valid = false;
          fault(i, "absolute position ambiguous after feedback gap; restart node to re-anchor");
        } else {
          double delta = fb->p - m.state.feedback.p;
          if (delta > period / 2) {delta -= period; --m.state.position_wraps;}
          else if (delta < -period / 2) {delta += period; ++m.state.position_wraps;}
          m.state.absolute_position += delta;
          m.tracking_time = received;
        }
      }
      m.state.feedback = *fb; m.last_feedback = now; ++m.state.feedback_count;
      if (fb->status > 1) {
        if (!m.state.device_fault) {m.last_stop = -1;}
        m.state.device_fault = true;
        if (!m.state.host_fault) {
          m.state.error = "motor fault code " + std::to_string(fb->status) + ": " + motor_status_text(fb->status);
        }
        m.state.phase = Phase::Fault;
        m.target = {};
      }
      else if (m.state.phase == Phase::Enabling && fb->status == 1) {m.state.phase = Phase::Enabled;}
      else if (m.state.phase == Phase::Stopping && fb->status == 0) {m.state.phase = Phase::Disabled;}
      else if (m.state.phase == Phase::Disabled && fb->status == 1) {fault(i, "unexpected external enable");}
      else if (m.state.phase == Phase::Enabled && fb->status == 0) {fault(i, "motor unexpectedly disabled");}
      if (feedback_callback) {feedback_callback(configs[i].name, m.state, received);}
      if (!queue.empty()) {
        const auto tx = queue.front();
        if (tx.motor == i && tx.kind == Kind::ModeWrite && tx.stage == 3 &&
          m.state.feedback_count > tx.feedback_before && fb->status == 0 &&
          std::abs(fb->v) <= std::max(0.1, m.state.mapping.v * 2 / 4095))
        {
          finish(OperationState::Confirmed, "mode readback and fresh disabled status confirmed",
            tx.raw, now);
          continue;
        }
        if (tx.motor == i && tx.stage == 2 && m.state.feedback_count > tx.feedback_before &&
          (tx.kind == Kind::Clear || tx.kind == Kind::Zero) && fb->status == 0 &&
          (tx.kind == Kind::Clear || std::abs(fb->p) <= m.state.mapping.p * 2 / 65535 + 1e-6))
        {
          if (tx.kind == Kind::Clear) {
            m.state.device_fault = false;
            if (!m.state.host_fault) {
              m.state.error.clear();
              m.state.phase = Phase::Disabled;
              m.target = {}; m.target.mode = m.state.mode;
            }
          }
          finish(OperationState::Confirmed,
            "fresh relevant feedback observed; no persistence guarantee", 0, now);
        }
      }
    }
  }
};

Bus::Bus(std::unique_ptr<Transport> transport, std::vector<MotorConfig> configs, bool simulated)
: impl_(std::make_unique<Impl>())
{
  (void)simulated;  // Both transports must pass identical startup confirmations.
  if (!transport || configs.empty() || configs.size() > 15) {
    throw std::invalid_argument("bus requires a transport and 1..15 motors");
  }
  std::set<std::string> names;
  std::set<unsigned> ids, command_ids;
  for (const auto & c : configs) {
    if (c.name.empty() || !names.insert(c.name).second || !ids.insert(c.id).second ||
      c.master >= 0x7ff) {throw std::invalid_argument("duplicate name/address or invalid master ID");}
    for (unsigned mode = 1; mode <= 4; ++mode) {command_ids.insert(control_id(c.id, Mode(mode)));}
    control_id(c.id, c.mode);
    resolve_limits(c.mapping, c.limits);
    for (double timeout : {c.command_timeout, c.feedback_timeout, c.watchdog}) {
      if (!std::isfinite(timeout) || timeout <= 0) {throw std::invalid_argument("timeouts must be positive");}
    }
    if (c.watchdog < 50e-6 || c.watchdog / 50e-6 > 4294967295.0) {
      throw std::invalid_argument("watchdog outside nonzero uint32 range");
    }
  }
  for (const auto & c : configs) {
    if (command_ids.count(c.master)) {
      throw std::invalid_argument("feedback master ID collides with a motor command ID");
    }
    Impl::Motor m;
    m.state.mode = c.mode; m.state.mapping = c.mapping;
    m.state.limits = resolve_limits(c.mapping, c.limits);
    m.target.mode = c.mode;
    impl_->motors.push_back(m);
  }
  impl_->transport = std::move(transport);
  impl_->configs = std::move(configs);
}
Bus::~Bus()
{
  if (!impl_) {return;}
  for (size_t i = 0; i < impl_->motors.size(); ++i) {
    try {impl_->stop(i);} catch (...) {}
  }
}
void Bus::tick(double now)
{
  auto & b = *impl_;
  if (!std::isfinite(now) || now < 0 || (b.last_tick >= 0 && now < b.last_tick)) {
    b.io_failure("invalid or non-monotonic bus clock"); return;
  }
  b.last_tick = now;
  if (b.failed) {
    // Keep the single receive owner alive for diagnostic feedback after an I/O fault.
    // Unknown mode/watchdog state remains latched until reconnect, never auto-enabled.
    try {
      for (const auto & f : b.transport->receive()) {b.receive(f, now);}
    } catch (...) {}
    for (size_t i = 0; i < b.motors.size(); ++i) {
      auto & m = b.motors[i];
      if (m.last_stop >= 0 && now - m.last_stop < 0.05) {continue;}
      for (unsigned mode = 1; mode <= 4; ++mode) {
        try {b.transport->send(special_frame(control_id(b.configs[i].id, Mode(mode)), 0xfd));}
        catch (...) {}
      }
      m.last_stop = now;
    }
    return;
  }
  try {
    for (const auto & f : b.transport->receive()) {b.receive(f, now);}
    if (b.failed) {return;}
    // Safety and stop traffic always precede parameter dispatch or ordinary control.
    for (size_t i = 0; i < b.motors.size(); ++i) {
      auto & m = b.motors[i];
      const auto & c = b.configs[i];
      if (m.state.absolute_position_valid &&
        (now - m.tracking_time) * m.state.mapping.v >=
        m.state.mapping.p * 65536 / 65535)
      {
        m.state.absolute_position_valid = false;
        b.fault(i, "absolute position ambiguous after feedback gap; restart node to re-anchor");
      }
      if (m.state.phase == Phase::Enabled || m.state.phase == Phase::Enabling) {
        if (!b.fresh(i, now)) {b.fault(i, "feedback timeout");}
        else if (now - m.last_command > c.command_timeout) {b.fault(i, "command publisher timeout");}
        else if (m.state.phase == Phase::Enabling && now - m.transition > c.feedback_timeout) {
          b.fault(i, "enable acknowledgement timeout");
        }
      } else if (m.state.phase == Phase::Stopping && now - m.transition > c.feedback_timeout) {
        b.fault(i, "disable acknowledgement timeout");
      }
    }
    for (size_t i = 0; i < b.motors.size(); ++i) {
      auto & m = b.motors[i];
      if (m.state.phase == Phase::Fault || m.state.phase == Phase::Stopping) {
        const bool retry_stop = m.state.host_fault || m.state.phase == Phase::Stopping;
        if (m.last_stop < 0 || (retry_stop && now - m.last_stop >= 0.05)) {
          b.stop(i); m.last_stop = now;
        }
      }
    }
    if (!b.queue.empty() && b.queue.front().kind == Impl::Kind::Save &&
      b.queue.front().stage == 3 && now - b.queue.front().sent >= 0.03)
    {
      b.finish(OperationState::Confirmed,
        "save acknowledged; power-cycle verification required for persistence", 0, now);
    }
    b.startup(now);
    b.transaction(now);
    for (size_t i = 0; i < b.motors.size(); ++i) {
      auto & m = b.motors[i];
      if (m.state.phase == Phase::Enabled) {
        b.transport->send(target_frame(b.configs[i].id, m.target, m.state.mapping, b.configs[i].limits));
      } else {
        // During register transactions, do not provoke ambiguous normal feedback.
        const bool parameter_active = !b.queue.empty() &&
          (!b.initialized || b.queue.front().stage != 0) &&
          b.queue.front().kind != Impl::Kind::Clear && b.queue.front().kind != Impl::Kind::Zero;
        if (!parameter_active && now >= b.quiet_until) {
          b.transport->send(query_frame(b.configs[i].id));
        }
      }
    }
  } catch (const std::exception & e) {b.io_failure(std::string("bus I/O/protocol failure: ") + e.what());}
}
MotorSnapshot Bus::snapshot(const std::string & name, double now) const
{
  const auto i = impl_->index(name);
  auto result = impl_->motors[i].state;
  result.feedback_age = impl_->motors[i].last_feedback < 0 ? -1 :
    now - impl_->motors[i].last_feedback;
  if (result.absolute_position_valid &&
    (now - impl_->motors[i].tracking_time) * result.mapping.v >=
    result.mapping.p * 65536 / 65535)
  {
    result.absolute_position_valid = false;
  }
  return result;
}
void Bus::enable(const std::string & name, double now)
{
  auto & b = *impl_; const auto i = b.index(name); auto & m = b.motors[i];
  if (!b.initialized || b.failed || !b.queue.empty() || !b.transport->activity_available() ||
    m.state.phase != Phase::Disabled || !b.fresh(i, now) || m.state.feedback.status != 0 ||
    !snapshot(name, now).absolute_position_valid)
  {
    throw std::runtime_error("enable requires ready bus, no maintenance, fresh disabled feedback and no latched fault");
  }
  m.target = {}; m.target.mode = m.state.mode;
  if (m.state.mode == Mode::PositionVelocity || m.state.mode == Mode::PositionForce) {
    m.target.position = m.state.absolute_position;
  }
  target_frame(b.configs[i].id, m.target, m.state.mapping, b.configs[i].limits);
  m.last_command = now; m.transition = now; m.state.phase = Phase::Enabling;
  try {
    b.transport->send(target_frame(b.configs[i].id, m.target, m.state.mapping, b.configs[i].limits));
    b.transport->send(special_frame(control_id(b.configs[i].id, m.state.mode), 0xfc));
  } catch (const std::exception & e) {b.io_failure(e.what()); throw;}
}
void Bus::transport_failed(const std::string & reason)
{
  if (!impl_->failed) {impl_->io_failure(reason);}
}
void Bus::disable(const std::string & name, double now)
{
  auto & b = *impl_; const auto i = b.index(name); auto & m = b.motors[i];
  m.target = {};
  const bool already_disabled = m.state.phase == Phase::Disabled &&
    b.fresh(i, now) && m.state.feedback.status == 0;
  if (m.state.phase != Phase::Fault && !already_disabled) {
    m.state.phase = Phase::Stopping; m.transition = now;
  }
  try {b.stop(i); m.last_stop = now;}
  catch (const std::exception & e) {b.io_failure(e.what());}
}
void Bus::reset(const std::string & name, double now)
{
  auto & b = *impl_; const auto i = b.index(name); auto & m = b.motors[i];
  if (!b.initialized || b.failed || !b.queue.empty() || m.state.phase != Phase::Fault ||
    !b.fresh(i, now) || m.state.feedback.status != 0 || !snapshot(name, now).absolute_position_valid)
  {
    throw std::runtime_error("reset requires initialized bus, valid absolute position and fresh disabled feedback");
  }
  m.state.phase = Phase::Disabled; m.state.error.clear(); m.target = {};
  m.state.host_fault = false; m.state.device_fault = false;
}
void Bus::fail(const std::string & name, const std::string & reason)
{
  impl_->fault(impl_->index(name), reason);
}
void Bus::set_feedback_callback(
  std::function<void(const std::string &, const MotorSnapshot &, double)> callback)
{
  impl_->feedback_callback = std::move(callback);
}
void Bus::command(const std::string & name, const Target & target, double now)
{
  auto & b = *impl_; const auto i = b.index(name); auto & m = b.motors[i];
  if (m.state.phase != Phase::Enabled || target.mode != m.state.mode) {
    throw std::runtime_error("target requires confirmed enabled motor and matching mode");
  }
  if (!std::isfinite(now) || now < b.last_tick || !b.fresh(i, now) ||
    !snapshot(name, now).absolute_position_valid) {
    throw std::runtime_error("target requires monotonic time and fresh feedback");
  }
  target_frame(b.configs[i].id, target, m.state.mapping, b.configs[i].limits);
  m.target = target; m.last_command = now;
}
uint64_t Bus::read_register(const std::string & name, uint8_t rid, double now)
{
  auto & b = *impl_; const auto i = b.index(name);
  std::string rejection;
  try {info(rid);} catch (const std::exception & e) {rejection = e.what();}
  return b.enqueue(i, Impl::Kind::Read, rid, 0, now, rejection);
}
uint64_t Bus::write_register(const std::string & name, uint8_t rid, uint32_t raw, double now)
{
  auto & b = *impl_; const auto i = b.index(name);
  std::string rejection;
  try {
    validate_raw(rid, raw);
    if (rid == 7 || rid == 8 || rid == 9 || rid == 10 || (rid >= 21 && rid <= 23) ||
      rid == 35) {throw std::invalid_argument("reserved maintenance register; specialized path required");}
    // Protection and motor-identity constants need an audited commissioning workflow.
    if (rid <= 6 || rid == 29 || rid == 30) {
      throw std::invalid_argument("high-risk commissioning write not supported by generic service");
    }
  } catch (const std::exception & e) {rejection = e.what();}
  return b.enqueue(i, Impl::Kind::Write, rid, raw, now, rejection);
}
uint64_t Bus::save_parameters(const std::string & name, double now)
{
  return impl_->enqueue(impl_->index(name), Impl::Kind::Save, 1, 0, now);
}
uint64_t Bus::set_mode(const std::string & name, Mode mode, double now)
{
  auto & b = *impl_; const auto i = b.index(name);
  std::string rejection;
  try {control_id(b.configs[i].id, mode);} catch (const std::exception & e) {rejection = e.what();}
  if (std::abs(b.motors[i].state.feedback.v) >
    std::max(0.1, b.motors[i].state.mapping.v * 2 / 4095))
  {rejection = "mode change requires fresh low speed";}
  return b.enqueue(i, Impl::Kind::ModeWrite, 10, unsigned(mode), now, rejection);
}
uint64_t Bus::clear_motor_fault(const std::string & name, double now)
{
  return impl_->enqueue(impl_->index(name), Impl::Kind::Clear, 0, 0, now);
}
uint64_t Bus::set_device_zero(const std::string & name, double now)
{
  const auto i = impl_->index(name);
  const auto & m = impl_->motors[i];
  const std::string rejection = std::abs(m.state.feedback.v) >
    std::max(0.1, m.state.mapping.v * 2 / 4095) ? "zero requires fresh low speed" : "";
  return impl_->enqueue(i, Impl::Kind::Zero, 0, 0, now, rejection);
}
RegisterOperation Bus::operation(uint64_t id) const
{
  const auto it = impl_->history.find(id);
  if (it == impl_->history.end()) {
    return {id, OperationState::Unknown, "", 0, 0, "operation unknown or evicted from bounded history"};
  }
  return it->second;
}
const std::vector<MotorConfig> & Bus::configs() const {return impl_->configs;}
bool Bus::ready() const {return impl_->initialized && !impl_->failed;}
std::string Bus::initialization_error() const {return impl_->init_error;}

struct SimBusTransport::Impl
{
  struct Motor
  {
    MotorConfig config;
    Feedback feedback{0, 0, 0, 0, 28, 29};
    std::map<uint8_t, uint32_t> registers, saved;
  };
  std::map<uint8_t, Motor> motors;
  std::vector<Frame> pending;
  bool drop{false};
  void feedback(Motor & m)
  {
    if (!drop) {
      auto wire = m.feedback;
      const double pmax = m.config.mapping.p;
      const double period = 2 * pmax * 65536 / 65535;
      double wrapped = std::fmod(wire.p + pmax, period);
      if (wrapped < 0) {wrapped += period;}
      wire.p = std::clamp(wrapped - pmax, -pmax, pmax);
      pending.push_back(feedback_frame(m.config.id, m.config.master, wire, m.config.mapping));
    }
  }
};
SimBusTransport::SimBusTransport(const std::vector<MotorConfig> & configs)
: impl_(std::make_unique<Impl>())
{
  for (const auto & c : configs) {
    Impl::Motor m; m.config = c;
    for (const auto & r : register_catalog()) {m.registers[r.rid] = 0;}
    m.registers[7] = c.master; m.registers[8] = c.id; m.registers[10] = unsigned(c.mode);
    m.registers[21] = bits(c.mapping.p); m.registers[22] = bits(c.mapping.v);
    m.registers[23] = bits(c.mapping.t); m.registers[20] = bits(10);
    m.registers[0] = bits(12); m.registers[1] = bits(1); m.registers[2] = bits(100);
    m.registers[3] = bits(0.5); m.registers[4] = bits(1); m.registers[5] = bits(-1);
    m.registers[6] = bits(100); m.registers[24] = bits(1000); m.registers[29] = bits(60);
    m.registers[30] = bits(1); m.registers[31] = bits(1); m.registers[32] = bits(100);
    m.registers[33] = bits(1000); m.registers[34] = bits(1000);
    m.registers[59] = bits(10); m.registers[60] = bits(24);
    m.registers[61] = bits(28); m.registers[62] = bits(29);
    m.saved = m.registers;
    if (!impl_->motors.emplace(c.id, m).second) {throw std::invalid_argument("duplicate simulated ID");}
  }
}
SimBusTransport::~SimBusTransport() = default;
void SimBusTransport::send(const Frame & f)
{
  auto & sim = *impl_;
  if (f.channel != 0 || (f.brs && !f.fd)) {throw std::invalid_argument("invalid simulated frame attributes");}
  if (f.id == 0x7ff && (f.length == 8 || f.length == 4) && f.data[1] == 0) {
    auto it = sim.motors.find(f.data[0]); if (it == sim.motors.end()) {return;}
    auto & m = it->second;
    if (f.data[2] == 0xcc) {sim.feedback(m); return;}
    if (f.data[2] == 0xaa && f.data[3] == 1 && m.feedback.status == 0) {
      m.saved = m.registers;
      Frame reply = f; reply.id = m.config.master;
      if (!sim.drop) {sim.pending.push_back(reply);}
      return;
    }
    auto r = m.registers.find(f.data[3]); if (r == m.registers.end()) {return;}
    if (f.data[2] == 0x55) {
      if (m.feedback.status != 0 || f.length != 8) {return;}
      try {validate_raw(f.data[3], raw_value(f));} catch (...) {return;}
      r->second = raw_value(f);
      if (f.data[3] == 10) {m.config.mode = Mode(r->second);}
      if (f.data[3] == 21) {m.config.mapping.p = number(r->second);}
      if (f.data[3] == 22) {m.config.mapping.v = number(r->second);}
      if (f.data[3] == 23) {m.config.mapping.t = number(r->second);}
    } else if (f.data[2] != 0x33) {return;}
    if (f.data[3] == 80 || f.data[3] == 81) {r->second = bits(m.feedback.p);}
    Frame reply = f; reply.id = m.config.master; reply.length = 8;
    put_raw(reply, 4, r->second);
    if (!sim.drop) {sim.pending.push_back(reply);}
    return;
  }
  for (auto & entry : sim.motors) {
    auto & m = entry.second;
    if (f.id != control_id(m.config.id, m.config.mode)) {continue;}
    if (f.length == 8 && std::all_of(f.data.begin(), f.data.begin() + 7,
      [](uint8_t value) {return value == 255;}))
    {
      if (f.data[7] == 0xfc && m.feedback.status == 0) {m.feedback.status = 1;}
      if (f.data[7] == 0xfd && m.feedback.status <= 1) {
        m.feedback.status = 0; m.feedback.v = 0; m.feedback.t = 0;
      }
      if (f.data[7] == 0xfb && m.feedback.status != 1) {m.feedback.status = 0;}
      if (f.data[7] == 0xfe && m.feedback.status == 0) {m.feedback.p = 0;}
    } else if (m.feedback.status == 1) {
      const double dt = 0.005;
      if (m.config.mode == Mode::Mit) {
        auto c = decode_mit(f, m.config.mapping);
        m.feedback.t = std::clamp(c.kp * (c.p - m.feedback.p) +
          c.kd * (c.v - m.feedback.v) + c.t, -m.config.mapping.t, m.config.mapping.t);
        m.feedback.v = std::clamp(m.feedback.v +
          (m.feedback.t - 0.2 * m.feedback.v) * dt, -m.config.mapping.v, m.config.mapping.v);
      } else if (m.config.mode == Mode::Velocity && f.length == 4) {
        Frame value = f;
        std::copy_n(f.data.begin(), 4, value.data.begin() + 4);
        m.feedback.v = std::clamp(number(raw_value(value)), -m.config.mapping.v, m.config.mapping.v);
        m.feedback.t = 0;
      } else if (f.length == 8) {
        Frame value = f; std::copy_n(f.data.begin(), 4, value.data.begin() + 4);
        const double position = number(raw_value(value));
        const double speed = m.config.mode == Mode::PositionVelocity ? number(raw_value(f)) :
          double(f.data[4] | (f.data[5] << 8)) / 100;
        m.feedback.v = std::clamp((position - m.feedback.p) / dt, -speed, speed);
        m.feedback.t = 0;
      }
      m.feedback.p += m.feedback.v * dt;
    }
    sim.feedback(m);
  }
}
std::vector<Frame> SimBusTransport::receive()
{
  auto frames = std::move(impl_->pending); impl_->pending.clear(); return frames;
}
void SimBusTransport::inject_fault(uint8_t id, uint8_t status)
{
  if (status > 15) {throw std::invalid_argument("fault status must fit four bits");}
  auto & m = impl_->motors.at(id); m.feedback.status = status; impl_->feedback(m);
}
void SimBusTransport::drop_replies(bool drop) {impl_->drop = drop;}
void SimBusTransport::inject_frame(const Frame & frame) {impl_->pending.push_back(frame);}
void SimBusTransport::set_position(uint8_t id, double position)
{
  auto & m = impl_->motors.at(id);
  if (!std::isfinite(position) || m.feedback.status != 0) {
    throw std::invalid_argument("simulated position requires finite value and disabled motor");
  }
  m.feedback.p = position;
}
void SimBusTransport::power_cycle(uint8_t id)
{
  auto & m = impl_->motors.at(id);
  m.registers = m.saved;
  m.config.mode = Mode(m.registers.at(10));
  m.config.master = m.registers.at(7);
  m.config.mapping = {number(m.registers.at(21)), number(m.registers.at(22)),
    number(m.registers.at(23))};
  m.feedback = {0, 0, 0, 0, 28, 29};
  impl_->pending.clear();
}
uint32_t SimBusTransport::register_value(uint8_t id, uint8_t rid) const
{
  return impl_->motors.at(id).registers.at(rid);
}
std::unique_ptr<Transport> make_sim_transport(const std::vector<MotorConfig> & motors)
{
  return std::make_unique<SimBusTransport>(motors);
}
}  // namespace dm_motor
