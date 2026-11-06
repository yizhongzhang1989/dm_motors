#include "dm_motor_control/can_transport.hpp"
#include "can_bus/msg/frame.hpp"
#include "can_bus/msg/frame_filter.hpp"
#include "can_bus/msg/tx_frame.hpp"
#include "can_bus/msg/heartbeat.hpp"
#include "can_bus/msg/status.hpp"
#include "can_bus/msg/optional_read_removed.hpp"
#include "can_bus/srv/register_client.hpp"
#include "can_bus/srv/maintenance.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
namespace dm_motor
{
namespace
{
using Register = can_bus::srv::RegisterClient;
using Maintenance = can_bus::srv::Maintenance;
using Wire = can_bus::msg::Frame;
using Removed = can_bus::msg::OptionalReadRemoved;
bool parameter(const Frame & f)
{
  return f.id == 0x7ff && f.length >= 3 &&
    (f.data[2] == 0x33 || f.data[2] == 0x55 || f.data[2] == 0xaa);
}
bool stop(const Frame & f)
{
  return f.length == 8 && f.data[7] == 0xfd &&
    std::all_of(f.data.begin(), f.data.begin() + 7, [](uint8_t b) {return b == 255;});
}
bool maintenance_frame(const Frame & f)
{
  return parameter(f) || (f.length == 8 &&
    (f.data[7] == 0xfb || f.data[7] == 0xfe) &&
    std::all_of(f.data.begin(), f.data.begin() + 7, [](uint8_t b) {return b == 255;}));
}
bool query(const Frame & f)
{
  return f.id == 0x7ff && f.length == 8 && f.data[1] == 0 && f.data[2] == 0xcc;
}
bool active(const Frame & f) {return !stop(f) && !maintenance_frame(f) && f.id != 0x7ff;}
Wire encode(const Frame & f)
{
  Wire w; w.can_id = f.id; w.channel = f.channel; w.fd = f.fd; w.brs = f.brs;
  w.length = f.length; w.data.assign(f.data.begin(), f.data.begin() + f.length);
  return w;
}
can_bus::msg::FrameFilter filter(uint32_t id, std::vector<uint8_t> prefix = {},
  std::vector<uint8_t> mask = {})
{
  can_bus::msg::FrameFilter result;
  result.can_id = id; result.mask = 0x7ff;
  result.prefix.assign(prefix.begin(), prefix.end());
  result.prefix_mask.assign(mask.begin(), mask.end()); return result;
}
std::string safe_name(std::string name)
{
  for (auto & c : name) {
    if (!(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') &&
      !(c >= '0' && c <= '9')) {c = '_';}
  }
  if (name.size() > 64) {name.resize(64);}
  return "motor_client_" + name;
}
rclcpp::ExecutorOptions executor_options(rclcpp::Node & node)
{
  rclcpp::ExecutorOptions options;
  options.context = node.get_node_base_interface()->get_context();
  return options;
}
}
struct CanTransport::Impl
{
  mutable std::mutex mutex;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  rclcpp::Publisher<can_bus::msg::TxFrame>::SharedPtr tx;
  rclcpp::Publisher<can_bus::msg::Heartbeat>::SharedPtr heartbeat;
  rclcpp::Subscription<can_bus::msg::Status>::SharedPtr status_sub;
  rclcpp::Subscription<Wire>::SharedPtr rx_sub;
  rclcpp::Subscription<Wire>::SharedPtr position_tx_sub;
  rclcpp::Subscription<Removed>::SharedPtr position_removed_sub;
  rclcpp::Client<Register>::SharedPtr registration;
  rclcpp::Client<Maintenance>::SharedPtr maintenance;
  rclcpp::TimerBase::SharedPtr timer;
  Register::Request request;
  std::deque<std::pair<Frame, double>> outgoing;
  std::deque<Frame> incoming;
  std::function<void()> receive_notification;
  std::string token, group, failure, physical_backend, owner;
  uint64_t epoch{0}, seen_epoch{0}, sequence{0}, heartbeat_sequence{0};
  uint64_t last_stop_sequence{0};
  uint64_t quiet_sequence{0};
  uint8_t id;
  uint16_t master;
  bool registered{false}, registering{false}, status_ready{false}, quiet{true}, fault{false};
  bool all_quiet{false};
  bool quiet_confirmed{false};
  bool endpoint_ready{false}, endpoint_seen{false};
  bool broker_rx_ready{false};
  bool heartbeat_confirmed{false};
  bool receive_verified{false};
  bool require_virtual{false};
  bool allow_native_observation{false};
  struct PositionRead
  {
    Frame frame;
    uint64_t id, published_steady, expires_steady, observed_steady{0};
    bool ambiguous{false};
    std::string token{};
    uint64_t epoch{0}, sequence{0}, removed_steady{0}, removal_stop_sequence{0};
  };
  std::optional<PositionRead> position_read;
  std::vector<PositionReadDispatch> position_dispatches;
  std::vector<PositionReadRemoval> position_removals;
  double last_probe{0};
  bool activity_requires_stop{false}, stop_published{false};
  bool maintenance_pending{false}, maintenance_owned{false}, service_pending{false};
  bool fault_clear_pending{false};
  uint64_t fault_clear_sequence{0};
  bool releasing{false}, shutting_down{false};
  int64_t registration_id{0}, maintenance_id{0};
  double timeout, started, last_status{0}, last_heartbeat{0}, registration_started{0};
  double service_started{0}, maintenance_started{0}, last_parameter{0}, last_renewal{0};
  double next_maintenance_attempt{0};
  void fail(const std::string & reason)
  {
    if (failure.empty()) {
      failure = reason; outgoing.clear();
      position_read.reset(); position_dispatches.clear(); position_removals.clear();
    }
  }
  bool channels_ready() const
  {
    return registered && status_ready && endpoint_ready && broker_rx_ready && heartbeat_confirmed &&
      failure.empty() && rx_sub &&
      rx_sub->get_publisher_count() > 0 && tx->get_subscription_count() > 0 &&
      heartbeat->get_subscription_count() > 0 &&
      can_bus::steady_seconds() - last_status < std::max(timeout, .5);
  }
  bool ready_locked() const {return channels_ready() && receive_verified;}
  bool external_maintenance_locked() const
  {
    return !maintenance_owned && !owner.empty() && owner != token;
  }
  Impl(rclcpp::Node & parent, const std::string & bus_namespace,
    const std::string & client_name, uint8_t motor, uint16_t master_id,
    bool fd, double rate, double receive_timeout, const std::string & fault_group, bool virtual_only,
    bool native_observation)
  : executor(executor_options(parent)),
    group(fault_group), id(motor), master(master_id), require_virtual(virtual_only),
    allow_native_observation(native_observation), timeout(receive_timeout),
    started(can_bus::steady_seconds())
  {
    if (bus_namespace.empty() || bus_namespace.front() != '/' || motor == 0 || motor > 15 ||
      master_id > 0x7ff || !std::isfinite(rate) || rate <= 0 || rate > 5000 ||
      !std::isfinite(timeout) || timeout <= 0) {
      throw std::invalid_argument("invalid remote CAN bus, motor ID, rate or timeout");
    }
    std::string bus = bus_namespace;
    while (bus.size() > 1 && bus.back() == '/') {bus.pop_back();}
    static std::atomic<uint64_t> unique{0};
    rclcpp::NodeOptions options;
    options.context(parent.get_node_base_interface()->get_context());
    options.use_global_arguments(false);
    options.start_parameter_services(false);
    node = std::make_shared<rclcpp::Node>(
      safe_name(client_name) + "_" + std::to_string(++unique), "/", options);
    const auto qos = rclcpp::QoS(64).reliable();
    tx = node->create_publisher<can_bus::msg::TxFrame>(bus + "/tx", qos);
    heartbeat = node->create_publisher<can_bus::msg::Heartbeat>(bus + "/heartbeat", qos);
    registration = node->create_client<Register>(bus + "/register_client");
    maintenance = node->create_client<Maintenance>(bus + "/maintenance");
    request.client_name = client_name; request.fault_group = group;
    request.lease_duration.sec = 0; request.lease_duration.nanosec = 500000000;
    request.rate_hz = rate;
    // Runtime motor rates may increase to 1000 Hz without re-registering the bus client.
    request.max_tx_hz = std::min(10000.0, std::max(3000.0, rate * 3));
    request.filters_tx = {filter(0x7ff, {id, 0}, {255, 255})};
    for (unsigned mode = 0; mode < 4; ++mode) {
      const auto command_id = static_cast<uint16_t>(id + mode * 0x100);
      request.filters_tx.push_back(filter(command_id));
      auto stop_frame = special_frame(command_id, 0xfd); stop_frame.fd = fd;
      request.stop_frames.push_back(encode(stop_frame));
    }
    auto poll_frame = query_frame(id); poll_frame.fd = fd;
    request.poll_frames.push_back(encode(poll_frame));
    request.filters_rx = {filter(master, {id}, {15})};
    for (uint8_t opcode : {0x33, 0x55, 0xaa}) {
      request.filters_rx.push_back(filter(master, {id, 0, opcode}, {255, 255, 255}));
    }
    status_sub = node->create_subscription<can_bus::msg::Status>(bus + "/status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](can_bus::msg::Status::ConstSharedPtr s) {
        std::lock_guard<std::mutex> lock(mutex);
        last_status = can_bus::steady_seconds(); seen_epoch = s->epoch;
        physical_backend = s->backend; status_ready = s->ready; owner = s->maintenance_owner;
        if (require_virtual && (s->backend != "virtual" || !s->port.empty())) {
          fail("Virtual-only motor refuses a non-virtual CAN backend; no registration or device commands sent");
          status_ready = false;
          return;
        }
        endpoint_ready = s->endpoint_ready;
        if (endpoint_seen && !endpoint_ready && s->ready) {
          fail("CAN virtual endpoint disconnected; relaunch required");
        }
        endpoint_seen = endpoint_seen || endpoint_ready;
        all_quiet = s->all_quiet && quiet;
        if (registered && s->epoch != epoch) {fail("CAN bus restarted; relaunch required");}
        if (!s->ready) {fail("CAN bus unavailable: " + s->error);}
        if (!group.empty() && std::find(s->group_faults.begin(), s->group_faults.end(), group) !=
          s->group_faults.end()) {fail("CAN fault group latched; relaunch required: " + group);}
        if (registered) {
          quiet_confirmed = false;
          heartbeat_confirmed = false;
          for (size_t i = 0; i < s->client_tokens.size(); ++i) {
            if (s->client_tokens[i] == token && i < s->client_rx_ready.size()) {
              broker_rx_ready = s->client_rx_ready[i];
            }
            if (s->client_tokens[i] == token && i < s->heartbeat_sequences.size() &&
              i < s->client_quiet.size())
            {
              heartbeat_confirmed = s->heartbeat_sequences[i] > 0;
              quiet_confirmed = s->client_quiet[i] && s->heartbeat_sequences[i] >= quiet_sequence;
            }
          }
          if (!quiet_confirmed) {all_quiet = false;}
          for (const auto & issue : s->client_errors) {
            if (issue.rfind(token + ": ", 0) == 0) {fail("CAN transmit rejected: " + issue);}
          }
          if (fault_clear_pending && fault_clear_sequence) {
            for (size_t i = 0; i < s->client_tokens.size(); ++i) {
              if (s->client_tokens[i] == token && i < s->heartbeat_sequences.size() &&
                i < s->client_faults.size() && !s->client_faults[i] &&
                s->heartbeat_sequences[i] >= fault_clear_sequence) {fault_clear_pending = false;}
            }
          }
        }
      });
    // Default-off clients incur no full-bus TX subscription or callback work.
    // Opt-in constructs the reader before native observations can be scheduled.
    if (allow_native_observation) {
      position_tx_sub = node->create_subscription<Wire>(bus + "/tx_observed",
        rclcpp::QoS(1024).reliable(), [this](Wire::ConstSharedPtr w) {position_tx_callback(*w);});
      position_removed_sub = node->create_subscription<Removed>(bus + "/optional_read_removed",
        rclcpp::QoS(64).reliable(), [this](Removed::ConstSharedPtr m) {position_removed_callback(*m);});
    }
    timer = node->create_wall_timer(std::chrono::milliseconds(5), [this] {tick();});
    executor.add_node(node);
    thread = std::thread([this] {
      try {executor.spin();}
      catch (const std::exception & e) {std::lock_guard<std::mutex> lock(mutex); fail(e.what());}
    });
  }
  void publish_heartbeat(double now)
  {
    can_bus::msg::Heartbeat h;
    h.token = token; h.epoch = epoch; h.sequence = ++heartbeat_sequence;
    h.sent_steady = static_cast<uint64_t>(now * 1e9);
    h.quiet = quiet; h.fault = fault; heartbeat->publish(h); last_heartbeat = now;
    if (fault_clear_pending && !fault && !fault_clear_sequence) {fault_clear_sequence = h.sequence;}
  }
  void request_maintenance(bool acquire, double now)
  {
    if (service_pending || !maintenance->service_is_ready()) {return;}
    auto req = std::make_shared<Maintenance::Request>();
    req->token = token; req->epoch = epoch; req->acquire = acquire;
    req->duration.nanosec = 600000000;
    service_pending = true; service_started = now; releasing = !acquire;
    auto future = maintenance->async_send_request(req,
      [this, acquire](rclcpp::Client<Maintenance>::SharedFuture result) {
        std::lock_guard<std::mutex> lock(mutex);
        service_pending = false;
        try {
          auto reply = result.get();
          if (!reply->success) {
            if (!acquire || maintenance_owned) {fail("CAN maintenance lease lost: " + reply->message);}
            next_maintenance_attempt = can_bus::steady_seconds() + .025;
            return;
          }
          if (acquire) {
            maintenance_owned = true; maintenance_pending = false;
            last_renewal = can_bus::steady_seconds();
            last_parameter = last_renewal;
          } else {
            maintenance_owned = false; releasing = false;
            maintenance_pending = std::any_of(outgoing.begin(), outgoing.end(),
              [](const auto & pending) {return maintenance_frame(pending.first);});
            if (maintenance_pending) {maintenance_started = can_bus::steady_seconds();}
          }
        } catch (const std::exception & e) {fail("CAN maintenance service: " + std::string(e.what()));}
      });
    maintenance_id = future.request_id;
  }
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double now = can_bus::steady_seconds();
    if (registered && now - last_heartbeat >= .05) {publish_heartbeat(now);}
    if (!failure.empty() || shutting_down) {return;}
    if (registered && now - last_status > std::max(timeout, .5)) {
      fail("CAN bus status timed out; relaunch required"); return;
    }
    if (registering && now - registration_started > 2) {
      registration->remove_pending_request(registration_id);
      fail("CAN client registration timed out; relaunch required"); return;
    }
    if (!registered && !registering) {
      if (now - started > 10) {fail("CAN bus registration service unavailable"); return;}
      if (!status_ready || !registration->service_is_ready() ||
        tx->get_subscription_count() == 0 || heartbeat->get_subscription_count() == 0) {return;}
      registering = true; registration_started = now;
      auto future = registration->async_send_request(std::make_shared<Register::Request>(request),
        [this](rclcpp::Client<Register>::SharedFuture result) {
          std::lock_guard<std::mutex> lock(mutex);
          registering = false;
          if (!failure.empty()) {return;}
          try {
            RCLCPP_DEBUG(node->get_logger(),
              "CAN registration reply after %.3f s; TX matches=%zu, heartbeat matches=%zu",
              can_bus::steady_seconds() - registration_started,
              tx->get_subscription_count(), heartbeat->get_subscription_count());
            const auto reply = result.get();
            if (!reply->success) {fail("CAN registration rejected: " + reply->message); return;}
            if (reply->epoch != seen_epoch) {fail("CAN epoch changed during registration"); return;}
            token = reply->token; epoch = reply->epoch; registered = true;
            rx_sub = node->create_subscription<Wire>(reply->rx_topic, rclcpp::QoS(64).reliable(),
              [this](Wire::ConstSharedPtr w) {receive_callback(*w);});
            publish_heartbeat(can_bus::steady_seconds());
          } catch (const std::exception & e) {fail("CAN registration failed: " + std::string(e.what()));}
        });
      registration_id = future.request_id;
      return;
    }
    if (!registered) {return;}
    if (!receive_verified) {
      if (now - started > 10) {
        fail("CAN initial read-only status round trip timed out; inspect device and RX/TX channels");
        return;
      }
      if (channels_ready() && now - last_probe >= .05)
      {
        can_bus::msg::TxFrame probe;
        probe.token = token; probe.epoch = epoch; probe.sequence = ++sequence;
        probe.valid_for.nanosec = 100000000;
        auto frame = query_frame(id); frame.fd = request.stop_frames.front().fd;
        probe.frame = encode(frame); probe.frame.header.stamp = node->now();
        tx->publish(probe); last_probe = now;
      }
    }
    if (service_pending && now - service_started > .4) {
      maintenance->remove_pending_request(maintenance_id);
      service_pending = false; fail("CAN maintenance service timed out"); return;
    }
    if (maintenance_pending && now - maintenance_started > 3) {
      fail("CAN maintenance requires every bus client fresh and quiet"); return;
    }
    if (maintenance_pending && now >= next_maintenance_attempt) {request_maintenance(true, now);}
    if (maintenance_owned && !releasing && now - last_renewal >= .2) {request_maintenance(true, now);}
    const bool parameters_queued = std::any_of(outgoing.begin(), outgoing.end(),
      [](const auto & p) {return maintenance_frame(p.first);});
    if (maintenance_owned && !parameters_queued && now - last_parameter >= .1 &&
      !service_pending) {request_maintenance(false, now);}
    flush_outgoing(now);
  }
  void flush_outgoing(double now)
  {
    // The runtime suppresses its own polls during ambiguous parameter transactions.
    // Other clients' maintenance must not starve this motor's absolute-position tracking.
    outgoing.erase(std::remove_if(outgoing.begin(), outgoing.end(), [&](const auto & pending) {
      return query(pending.first) && now - pending.second > .1;
    }), outgoing.end());
    if (outgoing.empty()) {return;}
    auto & pending = outgoing.front();
    if (now - pending.second > (maintenance_frame(pending.first) ? 3 : .1)) {
      fail("remote CAN send expired before transmission"); return;
    }
    const auto & f = pending.first;
    if (!stop(f) && !query(f) && (maintenance_pending || releasing ||
      external_maintenance_locked() || (maintenance_frame(f) && !maintenance_owned))) {return;}
    can_bus::msg::TxFrame m;
    m.token = token; m.epoch = epoch; m.sequence = ++sequence;
    m.valid_for.nanosec = 100000000;
    m.priority = stop(f) ? 1 : maintenance_frame(f) ? 2 : 0;
    m.active = active(f); m.sent_steady = static_cast<uint64_t>(pending.second * 1e9);
    m.frame = encode(f); m.frame.header.stamp = node->now(); tx->publish(m);
    if (stop(f)) {stop_published = true; last_stop_sequence = m.sequence;}
    if (maintenance_frame(f)) {last_parameter = now;}
    outgoing.pop_front();
  }
  void receive_callback(const Wire & w)
  {
    std::unique_lock<std::mutex> lock(mutex);
    if (!failure.empty() || w.can_id != master || w.length != 8 || w.data.size() != 8 ||
      w.channel || w.extended || w.rtr || w.error) {return;}
    const double now = can_bus::steady_seconds();
    if (!w.received_steady || now - w.received_steady * 1e-9 > timeout ||
      w.received_steady * 1e-9 > now + .01) {return;}
    // Parameter replies may share a feedback route with another motor's low nibble.
    const bool parameter_reply = w.data[1] == 0 &&
      (w.data[2] == 0x33 || w.data[2] == 0x55 || w.data[2] == 0xaa);
    if (!parameter_reply && (w.data[0] & 15) == id && last_probe > 0 &&
      w.received_steady * 1e-9 >= last_probe)
    {receive_verified = true;}
    if (parameter_reply && (quiet || !owner.empty() || maintenance_owned) && w.data[0] != id) {return;}
    const bool sampled_reply = position_read && w.data[0] == id && w.data[2] == 0x33 &&
      w.data[3] == position_read->frame.data[3] &&
      w.received_steady >= position_read->published_steady &&
      w.received_steady <= position_read->expires_steady;
    if (parameter_reply && w.data[0] == id && !sampled_reply) {last_parameter = now;}
    if (incoming.size() >= 64) {fail("remote CAN receive queue overflow"); return;}
    Frame f; f.id = w.can_id; f.length = w.length; f.fd = w.fd; f.brs = w.brs;
    f.received_steady = w.received_steady;
    std::copy(w.data.begin(), w.data.end(), f.data.begin()); incoming.push_back(f);
    const auto notification = receive_notification;
    lock.unlock();
    if (notification) {notification();}
  }
  void position_tx_callback(const Wire & w)
  {
    std::unique_lock<std::mutex> lock(mutex);
    if (!position_read || !failure.empty() || shutting_down) {return;}
    auto & read = *position_read;
    const auto & frame = read.frame;
    if (w.can_id != frame.id || w.channel != frame.channel || w.extended || w.rtr || w.error ||
      w.fd != frame.fd || w.brs != frame.brs || w.length != 8 || w.data.size() != 8 ||
      !std::equal(w.data.begin(), w.data.end(), frame.data.begin()) ||
      w.received_steady < read.published_steady || !w.received_steady)
    {return;}
    // This is broker TX acquisition evidence, not publication-return or a
    // device ACK. An identical later wire transaction is ambiguous: CAN has
    // no per-read nonce. At most one initial and one ambiguous event are kept.
    if (read.ambiguous || (read.observed_steady && read.observed_steady == w.received_steady)) {return;}
    const auto now_ns = static_cast<uint64_t>(can_bus::steady_seconds() * 1e9);
    read.ambiguous = read.observed_steady != 0 || read.removed_steady != 0 || w.received_steady > read.expires_steady ||
      w.received_steady > now_ns;
    read.observed_steady = w.received_steady;
    position_dispatches.push_back({read.id, w.received_steady, read.ambiguous});
    const auto notification = receive_notification;
    lock.unlock();
    if (notification) {notification();}
  }
  void position_removed_callback(const Removed & m)
  {
    std::unique_lock<std::mutex> lock(mutex);
    if (!position_read || !failure.empty() || shutting_down) {return;}
    auto & read = *position_read;
    // An old token/epoch/sequence can never cancel the current single flight.
    if (m.token != read.token || m.epoch != read.epoch || m.sequence != read.sequence ||
      token != read.token || epoch != read.epoch) {return;}
    if (read.ambiguous) {return;}
    const auto & w = m.frame;
    const auto & f = read.frame;
    const auto now_ns = static_cast<uint64_t>(can_bus::steady_seconds() * 1e9);
    const bool bad = m.sent_steady != read.published_steady || !m.removed_steady ||
      m.removed_steady < read.published_steady || m.removed_steady >= read.expires_steady ||
      m.removed_steady > now_ns || now_ns >= read.expires_steady ||
      m.stop_sequence <= read.sequence || m.stop_sequence > last_stop_sequence ||
      w.can_id != f.id || w.channel != f.channel || w.extended || w.rtr || w.error ||
      w.fd != f.fd || w.brs != f.brs || w.length != 8 || w.data.size() != 8 ||
      !std::equal(w.data.begin(), w.data.end(), f.data.begin()) || read.observed_steady ||
      (read.removed_steady && (read.removed_steady != m.removed_steady ||
      read.removal_stop_sequence != m.stop_sequence));
    if (!bad && read.removed_steady) {return;}  // Identical witness is idempotent.
    read.ambiguous = bad; read.removed_steady = m.removed_steady;
    read.removal_stop_sequence = m.stop_sequence;
    position_removals.push_back({read.id, m.removed_steady, bad});
    const auto notification = receive_notification;
    lock.unlock();
    if (notification) {notification();}
  }
  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      shutting_down = true;
      if (registered && node->get_node_base_interface()->get_context()->is_valid()) {
        // Exact registered stop bypasses ordinary fault/maintenance queues.
        can_bus::msg::TxFrame m;
        m.token = token; m.epoch = epoch; m.sequence = ++sequence;
        m.valid_for.nanosec = 100000000; m.priority = 1;
        for (const auto & f : request.stop_frames) {
          m.sequence = ++sequence; m.frame = f; m.frame.header.stamp = node->now();
          try {tx->publish(m);} catch (...) {}
        }
      }
    }
    executor.cancel();
    if (thread.joinable()) {thread.join();}
  }
};
CanTransport::CanTransport(rclcpp::Node & node, const std::string & bus_namespace,
  const std::string & client_name, uint8_t motor_id, uint16_t master_id, bool fd,
  double rate, double timeout, const std::string & fault_group, bool require_virtual,
  bool allow_native_position_observation)
: impl_(std::make_unique<Impl>(node, bus_namespace, client_name, motor_id, master_id,
    fd, rate, timeout, fault_group, require_virtual, allow_native_position_observation)) {}
CanTransport::~CanTransport() = default;
void CanTransport::send(const Frame & f)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const bool fd = impl_->request.stop_frames.front().fd;
  if (f.length > 8 || f.id > 0x7ff || f.channel || (f.brs && !f.fd) || (f.fd && !fd)) {
    throw std::invalid_argument("invalid DM remote CAN frame");
  }
  if (!impl_->ready_locked() && !stop(f)) {
    throw std::runtime_error(impl_->failure.empty() ? "CAN transport not registered/ready" : impl_->failure);
  }
  if (!stop(f) && !maintenance_frame(f) && f.id != 0x7ff &&
    (impl_->maintenance_owned || impl_->maintenance_pending || impl_->releasing ||
    !impl_->owner.empty() || impl_->fault || impl_->fault_clear_pending)) {
    throw std::runtime_error("motion blocked by CAN bus maintenance or fault reset acknowledgment");
  }
  if (maintenance_frame(f) && (!impl_->maintenance_owned || impl_->releasing) &&
    !impl_->maintenance_pending) {
    impl_->maintenance_pending = true; impl_->maintenance_started = can_bus::steady_seconds();
  }
  if (active(f)) {
    if (impl_->quiet) {
      impl_->quiet_sequence = impl_->heartbeat_sequence + 1;
      impl_->quiet_confirmed = false; impl_->all_quiet = false; impl_->last_heartbeat = 0;
    }
    impl_->quiet = false; impl_->activity_requires_stop = true; impl_->stop_published = false;
  }
  if (stop(f)) {
    // STOP priority/queue retirement is independent of diagnostic association.
    // Runtime hard cancellation clears it explicitly; a normal native80 retire
    // still needs a possibly delayed TX witness. Keep only that association:
    // never delay STOP, retransmit a read, or retract a broker-published request.
    auto & queue = impl_->outgoing;
    queue.erase(std::remove_if(queue.begin(), queue.end(), [&](const auto & p) {
      return !stop(p.first) || p.first.id == f.id;
    }), queue.end());
  }
  else if (!maintenance_frame(f)) {
    auto & queue = impl_->outgoing;
    queue.erase(std::remove_if(queue.begin(), queue.end(), [&](const auto & p) {
      return !maintenance_frame(p.first) && !stop(p.first) && p.first.id == f.id;
    }), queue.end());
  }
  if (impl_->outgoing.size() >= 32) {throw std::runtime_error("remote CAN transmit queue full");}
  auto wire_frame = f;
  wire_frame.fd = fd;
  impl_->outgoing.emplace_back(wire_frame, can_bus::steady_seconds());
  if (!maintenance_frame(f) && impl_->registered && !impl_->shutting_down &&
    impl_->failure.empty())
  {
    impl_->flush_outgoing(can_bus::steady_seconds());
  }
}
std::vector<Frame> CanTransport::receive()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->failure.empty()) {throw std::runtime_error(impl_->failure);}
  const auto now = can_bus::steady_seconds();
  std::vector<Frame> result;
  while (!impl_->incoming.empty()) {
    auto f = impl_->incoming.front(); impl_->incoming.pop_front();
    if (now - f.received_steady * 1e-9 <= impl_->timeout) {result.push_back(f);}
  }
  return result;
}
void CanTransport::set_receive_notification(std::function<void()> notification)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->receive_notification = std::move(notification);
}
bool CanTransport::ready() const
{std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->ready_locked();}
std::string CanTransport::error() const
{std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->failure;}
std::string CanTransport::backend() const
{std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->physical_backend;}
bool CanTransport::simulated() const {const auto name = backend(); return name == "sim" || name == "virtual";}
bool CanTransport::maintenance_available() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->ready_locked() || impl_->releasing || !impl_->quiet ||
    !impl_->quiet_confirmed) {return false;}
  return impl_->maintenance_owned ||
    (impl_->all_quiet && (impl_->owner.empty() || impl_->owner == impl_->token));
}
bool CanTransport::maintenance_blocked() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->registered || !impl_->status_ready || impl_->seen_epoch != impl_->epoch ||
    impl_->maintenance_owned || (impl_->quiet && !impl_->quiet_confirmed)) {return false;}
  return !impl_->all_quiet;
}
bool CanTransport::activity_available() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->ready_locked() && impl_->owner.empty() &&
    !impl_->maintenance_pending && !impl_->maintenance_owned &&
    !impl_->releasing && !impl_->service_pending && !impl_->fault &&
    !impl_->fault_clear_pending;
}
bool CanTransport::native_position_observation_supported() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->allow_native_observation && impl_->position_tx_sub &&
    !impl_->request.stop_frames.front().fd &&
    impl_->position_tx_sub->get_publisher_count() > 0;
}
uint64_t CanTransport::bus_epoch() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->registered ? impl_->epoch : 0;
}
bool CanTransport::try_send_native_position_read(const Frame & f, uint64_t read_id, uint64_t expires_steady)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->allow_native_observation || !impl_->position_tx_sub ||
    impl_->request.stop_frames.front().fd ||
    impl_->position_tx_sub->get_publisher_count() == 0) {return false;}
  const bool fd = impl_->request.stop_frames.front().fd;
  if (!read_id || f.id != 0x7ff || f.length != 8 || f.channel || f.brs || f.fd ||
    f.data[0] != impl_->id || f.data[1] != 0 || f.data[2] != 0x33 ||
    f.data[3] != 80 ||
    !std::all_of(f.data.begin() + 4, f.data.end(), [](uint8_t b) {return b == 0;}))
  {throw std::invalid_argument("position observation permits only own read-only p_m80 frames");}
  const auto now_ns = static_cast<uint64_t>(can_bus::steady_seconds() * 1e9);
  if (expires_steady <= now_ns || expires_steady - now_ns > 20000000) {return false;}
  if (!impl_->ready_locked() || impl_->shutting_down || impl_->position_read ||
    !impl_->outgoing.empty() || !impl_->owner.empty() || impl_->maintenance_pending ||
    impl_->maintenance_owned || impl_->releasing || impl_->service_pending ||
    impl_->fault || impl_->fault_clear_pending) {return false;}
  Frame wire = f; wire.fd = fd;
  can_bus::msg::TxFrame message;
  message.token = impl_->token; message.epoch = impl_->epoch; message.sequence = ++impl_->sequence;
  impl_->position_read = Impl::PositionRead{wire, read_id, now_ns, expires_steady};
  impl_->position_read->token = message.token; impl_->position_read->epoch = message.epoch;
  impl_->position_read->sequence = message.sequence;
  message.sent_steady = now_ns;
  message.valid_for.nanosec = static_cast<uint32_t>(expires_steady - now_ns);
  message.priority = 3; message.active = false;
  message.frame = encode(wire); message.frame.header.stamp = impl_->node->now();
  try {impl_->tx->publish(message);}
  catch (...) {impl_->position_read.reset(); throw;}
  // Publication only. Runtime must await the separate broker TX observation.
  // Never enqueue this read in the maintenance or ordinary target queue.
  return true;
}
std::vector<PositionReadDispatch> CanTransport::take_position_read_dispatches()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::vector<PositionReadDispatch> result;
  result.swap(impl_->position_dispatches);
  return result;
}
std::vector<PositionReadRemoval> CanTransport::take_position_read_removals()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::vector<PositionReadRemoval> result;
  result.swap(impl_->position_removals);
  return result;
}
void CanTransport::cancel_position_read(uint64_t read_id)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->position_read && impl_->position_read->id == read_id) {impl_->position_read.reset();}
  auto & events = impl_->position_dispatches;
  events.erase(std::remove_if(events.begin(), events.end(),
    [read_id](const auto & event) {return event.read_id == read_id;}), events.end());
  auto & removals = impl_->position_removals;
  removals.erase(std::remove_if(removals.begin(), removals.end(),
    [read_id](const auto & event) {return event.read_id == read_id;}), removals.end());
}
bool CanTransport::prepare_maintenance()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->ready_locked() || !impl_->quiet) {return false;}
  if (impl_->maintenance_owned && !impl_->releasing) {return true;}
  if (!impl_->maintenance_pending && !impl_->releasing && impl_->all_quiet &&
    (impl_->owner.empty() || impl_->owner == impl_->token))
  {
    impl_->maintenance_pending = true;
    impl_->maintenance_started = can_bus::steady_seconds();
    impl_->next_maintenance_attempt = impl_->maintenance_started;
  }
  return false;
}
void CanTransport::set_quiet(bool value)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (value && impl_->activity_requires_stop && !impl_->stop_published) {return;}
  if (impl_->quiet != value) {
    impl_->quiet_sequence = impl_->heartbeat_sequence + 1;
    impl_->quiet_confirmed = false;
    impl_->all_quiet = false;
    impl_->last_heartbeat = 0;
  }
  impl_->quiet = value;
  if (value) {impl_->activity_requires_stop = false;}
}
void CanTransport::set_fault(bool value)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->group.empty() && impl_->fault && !value) {
    impl_->fault_clear_pending = true; impl_->fault_clear_sequence = 0;
  }
  if (value) {impl_->fault_clear_pending = false; impl_->fault_clear_sequence = 0;}
  impl_->fault = impl_->group.empty() ? value : (impl_->fault || value);
}
void CanTransport::spin_shutdown()
{
  // The isolated executor remains alive even when the parent executor has stopped.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}
}
