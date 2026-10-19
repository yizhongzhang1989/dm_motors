#include "can_bus/router.hpp"
#include "can_bus/msg/frame.hpp"
#include "can_bus/msg/status.hpp"
#include "can_bus/msg/tx_frame.hpp"
#include "can_bus/msg/heartbeat.hpp"
#include "can_bus/srv/register_client.hpp"
#include "can_bus/srv/maintenance.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <random>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
namespace
{
using Wire = can_bus::msg::Frame;
can_bus::Frame decode(const Wire & m)
{
  can_bus::Frame f;
  f.id = m.can_id; f.channel = m.channel; f.extended = m.extended;
  f.rtr = m.rtr; f.error = m.error; f.fd = m.fd; f.brs = m.brs;
  // Payload size is authoritative for compatibility with ordinary ROS producers.
  if (m.data.size() > 64 || (m.length && m.length != m.data.size())) {
    throw std::invalid_argument("CAN payload length mismatch");
  }
  f.length = m.data.size();
  std::copy(m.data.begin(), m.data.end(), f.data.begin());
  f.received_steady = m.received_steady;
  return f;
}
Wire encode(const can_bus::Frame & f)
{
  Wire m;
  m.can_id = f.id; m.channel = f.channel; m.extended = f.extended;
  m.rtr = f.rtr; m.error = f.error; m.fd = f.fd; m.brs = f.brs;
  m.length = f.length; m.data.assign(f.data.begin(), f.data.begin() + f.length);
  m.received_steady = f.received_steady;
  return m;
}
can_bus::Filter filter(const can_bus::msg::FrameFilter & f)
{
  return {f.can_id, f.mask, f.extended, f.channel,
    {f.prefix.begin(), f.prefix.end()}, {f.prefix_mask.begin(), f.prefix_mask.end()}};
}
double seconds(const builtin_interfaces::msg::Duration & d)
{return d.sec + d.nanosec * 1e-9;}
uint64_t epoch()
{
  std::random_device random;
  return (uint64_t(random()) << 32) ^ random() ^
    uint64_t(std::chrono::steady_clock::now().time_since_epoch().count());
}
class BusNode final : public rclcpp::Node
{
  can_bus::Router router_;
  std::unique_ptr<can_bus::Transport> transport_;
  std::string backend_, error_;
  std::string port_;
  uint32_t bitrate_{1000000};
  bool ready_{false};
  uint64_t rx_{0}, tx_{0};
  rclcpp::Publisher<Wire>::SharedPtr rx_pub_, wire_tx_, tx_observed_;
  rclcpp::Publisher<can_bus::msg::Status>::SharedPtr status_;
  std::map<std::string, rclcpp::Publisher<Wire>::SharedPtr> client_rx_;
  std::map<std::string, std::string> client_errors_;
  rclcpp::Subscription<Wire>::SharedPtr wire_rx_;
  rclcpp::Subscription<can_bus::msg::TxFrame>::SharedPtr tx_sub_;
  rclcpp::Subscription<can_bus::msg::Heartbeat>::SharedPtr heartbeat_;
  rclcpp::Service<can_bus::srv::RegisterClient>::SharedPtr registration_;
  rclcpp::Service<can_bus::srv::Maintenance>::SharedPtr maintenance_;
  rclcpp::TimerBase::SharedPtr io_timer_, status_timer_;
  std::mutex observer_mutex_;
  std::condition_variable observer_condition_;
  std::deque<std::pair<rclcpp::Publisher<Wire>::SharedPtr, Wire>> observer_queue_;
  std::atomic<bool> observer_running_{false};
  std::atomic<uint64_t> observer_dropped_{0};
  std::atomic<uint32_t> observer_inflight_{0};
  std::thread observer_thread_;
  void observe(const rclcpp::Publisher<Wire>::SharedPtr & publisher, const Wire & frame)
  {
    if (!observer_running_ || !publisher->get_subscription_count()) {return;}
    {
      std::lock_guard<std::mutex> guard(observer_mutex_);
      if (observer_queue_.size() == 8192) {
        observer_queue_.pop_front(); ++observer_dropped_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "CAN observer backlog overflow; control RX is unaffected, inspect observer_dropped");
      }
      observer_queue_.emplace_back(publisher, frame);
    }
    observer_condition_.notify_one();
  }
  void publish_observations()
  {
    while (observer_running_) {
      decltype(observer_queue_) batch;
      {
        std::unique_lock<std::mutex> guard(observer_mutex_);
        observer_condition_.wait(guard, [this] {return !observer_running_ || !observer_queue_.empty();});
        if (!observer_running_) {return;}
        batch.swap(observer_queue_);
        observer_inflight_ = batch.size();
      }
      for (const auto & item : batch) {
        if (!observer_running_) {return;}
        try {item.first->publish(item.second);}
        catch (const std::exception & error) {
          ++observer_dropped_;
          if (rclcpp::ok(get_node_base_interface()->get_context())) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
              "CAN observation publication failed: %s", error.what());
          }
        }
        --observer_inflight_;
      }
    }
  }
  bool endpoint_ready() const
  {
    return ready_ && (backend_ != "sim" ||
      (wire_tx_->get_subscription_count() > 0 && wire_rx_->get_publisher_count() > 0));
  }
  void publish_status()
  {
    can_bus::msg::Status s;
    s.header.stamp = now(); s.epoch = router_.epoch; s.ready = ready_;
    s.endpoint_ready = endpoint_ready();
    s.backend = backend_; s.error = error_; s.rx_count = rx_; s.tx_count = tx_;
    s.port = port_; s.can_bitrate = bitrate_; s.can_fd = router_.allow_fd;
    s.rejected = router_.rejected; s.queue_depth = router_.queue_depth();
    s.observer_dropped = observer_dropped_;
    {
      std::lock_guard<std::mutex> guard(observer_mutex_);
      s.observer_queue_depth = observer_queue_.size() + observer_inflight_.load();
    }
    s.clients = router_.clients.size(); s.maintenance_owner = router_.maintenance_owner;
    s.all_quiet = std::all_of(router_.clients.begin(), router_.clients.end(),
      [](const auto & item) {
        const auto & c = item.second;
        return c.revoked || (c.heartbeat_sequence && c.quiet &&
          can_bus::steady_seconds() - c.last_heartbeat <= c.lease);
      });
    s.group_faults.assign(router_.group_faults.begin(), router_.group_faults.end());
    for (const auto & item : client_errors_) {s.client_errors.push_back(item.first + ": " + item.second);}
    for (const auto & item : router_.clients) {
      if (item.second.revoked) {
        s.client_errors.push_back(item.first + ": client lease revoked; relaunch required");
      }
      s.client_names.push_back(item.second.name);
      s.client_tokens.push_back(item.first);
      s.heartbeat_sequences.push_back(item.second.heartbeat_sequence);
      s.client_quiet.push_back(item.second.heartbeat_sequence && item.second.quiet);
      s.client_faults.push_back(item.second.fault);
      const auto publisher = client_rx_.find(item.first);
      s.client_rx_ready.push_back(item.second.rx.empty() ||
        (publisher != client_rx_.end() && publisher->second->get_subscription_count() > 0));
    }
    status_->publish(s);
  }
  void dispatch(can_bus::Frame f)
  {
    can_bus::validate_frame(f, router_.allow_fd);
    if (!transport_ || !f.received_steady) {
      f.received_steady = static_cast<uint64_t>(can_bus::steady_seconds() * 1e9);
    }
    auto m = encode(f); m.header.stamp = now(); ++rx_;
    for (const auto & item : router_.clients) {
      const auto & c = item.second;
      if (!c.revoked && std::any_of(c.rx.begin(), c.rx.end(),
        [&](const auto & route) {return can_bus::matches(route, f);})) {
        client_rx_.at(item.first)->publish(m);
      }
    }
    observe(rx_pub_, m);
  }
  void transmit(const can_bus::Frame & f)
  {
    if (transport_) {transport_->send(f);}
    else {auto m = encode(f); m.header.stamp = now(); wire_tx_->publish(m);}
    ++tx_;
    auto observed = encode(f);
    observed.header.stamp = now();
    observed.received_steady = static_cast<uint64_t>(can_bus::steady_seconds() * 1e9);
    observe(tx_observed_, observed);
  }
  void drain_transmit()
  {
    for (auto & item : router_.clients) {
      item.second.receive_ready = item.second.rx.empty() ||
        client_rx_.at(item.first)->get_subscription_count() > 0;
    }
    for (const auto & frame : router_.drain(can_bus::steady_seconds(), endpoint_ready())) {transmit(frame);}
  }
  void fail_bus(const std::exception & error)
  {
    error_ = error.what(); ready_ = false; router_.shutdown();
    RCLCPP_ERROR(get_logger(), "CAN bus stopped: %s", error.what());
    for (const auto & frame : router_.drain(can_bus::steady_seconds())) {
      try {transmit(frame);}
      catch (const std::exception & stop_error) {
        RCLCPP_ERROR(get_logger(), "CAN emergency STOP transmission failed: %s", stop_error.what());
      }
    }
    publish_status();
  }
  void flush_transmit()
  {
    if (!ready_) {return;}
    try {drain_transmit();}
    catch (const std::exception & error) {fail_bus(error);}
  }
  void tick()
  {
    if (!ready_) {return;}
    try {
      drain_transmit();
      if (transport_) {for (const auto & f : transport_->receive()) {dispatch(f);}}
    } catch (const std::exception & e) {
      fail_bus(e);
    }
  }
public:
  BusNode() : Node("can_bus"), router_(epoch())
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    backend_ = declare_parameter<std::string>("backend", "sim", startup);
    const auto port = declare_parameter<std::string>("port", "", startup);
    port_ = port;
    router_.allow_fd = declare_parameter<bool>("can_fd", false, startup);
    const auto bitrate = declare_parameter<int>("can_bitrate", 1000000, startup);
    if (bitrate <= 0 || bitrate > 5000000) {
      throw std::invalid_argument("can_bitrate must be 1..5000000 bits per second");
    }
    bitrate_ = static_cast<uint32_t>(bitrate);
    const auto rate = declare_parameter<double>("io_rate", 1000, startup);
    declare_parameter<double>("client_timeout", .5, startup);
    if (!std::isfinite(rate) || rate < 10 || rate > 10000) {
      throw std::invalid_argument("io_rate outside [10,10000]");
    }
    const auto qos = rclcpp::QoS(64).reliable();
    rx_pub_ = create_publisher<Wire>("rx", qos);
    tx_observed_ = create_publisher<Wire>("tx_observed", qos);
    wire_tx_ = create_publisher<Wire>("wire_tx", qos);
    status_ = create_publisher<can_bus::msg::Status>("status", rclcpp::QoS(1).reliable().transient_local());
    if (backend_ == "sim") {
      wire_rx_ = create_subscription<Wire>("wire_rx", qos, [this](Wire::ConstSharedPtr m) {
        if (!ready_) {return;}
        try {dispatch(decode(*m));} catch (...) {++router_.rejected;}
      });
    }
    tx_sub_ = create_subscription<can_bus::msg::TxFrame>("tx", qos,
      [this](can_bus::msg::TxFrame::ConstSharedPtr m) {
        if (!ready_) {++router_.rejected; return;}
        bool accepted = false;
        try {
          const double ttl = seconds(m->valid_for);
          const double age = (now() - rclcpp::Time(m->frame.header.stamp, get_clock()->get_clock_type())).seconds();
          if ((m->frame.header.stamp.sec == 0 && m->frame.header.stamp.nanosec == 0) ||
            age < -.01 || age >= ttl) {throw std::invalid_argument("stale or unstamped TX");}
          const auto f = decode(m->frame);
          if (m->active && (!m->sent_steady ||
            can_bus::steady_seconds() - m->sent_steady * 1e-9 > ttl ||
            m->sent_steady * 1e-9 > can_bus::steady_seconds() + .01)) {
            throw std::invalid_argument("missing or stale activity timestamp");
          }
          if (backend_ == "serial") {can_bus::serial_packet(can_bus::legacy_serial_frame(f));}
          router_.enqueue(m->token, m->epoch, m->sequence, ttl - std::max(0.0, age),
            m->priority, f, can_bus::steady_seconds(), m->active, m->sent_steady);
          accepted = true;
        }
        catch (const std::exception & e) {
          ++router_.rejected;
          const auto client = router_.clients.find(m->token);
          if (m->active && m->epoch == router_.epoch && client != router_.clients.end()) {
            client_errors_[m->token] = e.what();
          }
        }
        if (accepted) {flush_transmit();}
      });
    heartbeat_ = create_subscription<can_bus::msg::Heartbeat>("heartbeat", qos,
      [this](can_bus::msg::Heartbeat::ConstSharedPtr m) {
        try {
          auto found = router_.clients.find(m->token);
          const auto clock = can_bus::steady_seconds();
          if (!m->sent_steady || found == router_.clients.end() ||
            clock - m->sent_steady * 1e-9 > found->second.lease ||
            m->sent_steady * 1e-9 > clock + .01) {
            throw std::invalid_argument("missing or stale heartbeat timestamp");
          }
          router_.heartbeat(m->token, m->epoch, m->sequence,
            m->quiet, m->fault, clock, m->sent_steady);
          flush_transmit();
        }
        catch (...) {++router_.rejected;}
      });
    registration_ = create_service<can_bus::srv::RegisterClient>("register_client",
      [this](const std::shared_ptr<can_bus::srv::RegisterClient::Request> req,
      std::shared_ptr<can_bus::srv::RegisterClient::Response> res) {
        const double started = can_bus::steady_seconds();
        try {
          if (!ready_) {throw std::runtime_error("bus unavailable: " + error_);}
          can_bus::Client c; c.name = req->client_name; c.group = req->fault_group;
          c.lease = seconds(req->lease_duration); c.max_tx_hz = req->max_tx_hz;
          for (const auto & f : req->filters_rx) {c.rx.push_back(filter(f));}
          for (const auto & f : req->filters_tx) {c.tx.push_back(filter(f));}
          for (const auto & f : req->stop_frames) {
            auto frame = decode(f);
            if (backend_ == "serial") {can_bus::serial_packet(can_bus::legacy_serial_frame(frame));}
            c.stops.push_back(frame);
          }
          for (const auto & f : req->poll_frames) {
            auto frame = decode(f);
            if (backend_ == "serial") {can_bus::serial_packet(can_bus::legacy_serial_frame(frame));}
            c.polls.push_back(frame);
          }
          res->token = router_.add(std::move(c), can_bus::steady_seconds());
          for (auto it = client_rx_.begin(); it != client_rx_.end();) {
            if (!router_.clients.count(it->first)) {
              client_errors_.erase(it->first); it = client_rx_.erase(it);
            } else {++it;}
          }
          const std::string relative = "clients/" + res->token + "/rx";
          client_rx_[res->token] = create_publisher<Wire>(relative, rclcpp::QoS(64).reliable());
          res->rx_topic = client_rx_[res->token]->get_topic_name();
          res->epoch = router_.epoch; res->success = true;
          const double prepared = can_bus::steady_seconds();
          router_.clients.at(res->token).last_heartbeat = prepared;
          if (prepared - started > seconds(req->lease_duration) / 2) {
            RCLCPP_WARN(get_logger(), "CAN registration %s took %.3f s; lease starts after RX setup",
              req->client_name.c_str(), prepared - started);
          }
        } catch (const std::exception & e) {res->message = e.what();}
      });
    maintenance_ = create_service<can_bus::srv::Maintenance>("maintenance",
      [this](const std::shared_ptr<can_bus::srv::Maintenance::Request> req,
      std::shared_ptr<can_bus::srv::Maintenance::Response> res) {
        try {
          const auto publisher = client_rx_.find(req->token);
          const auto client = router_.clients.find(req->token);
          const bool client_ready = client != router_.clients.end() &&
            (client->second.rx.empty() || (publisher != client_rx_.end() &&
            publisher->second->get_subscription_count() > 0));
          res->success = ready_ && (!req->acquire || (endpoint_ready() && client_ready)) &&
            router_.maintenance(req->token, req->epoch,
            req->acquire, seconds(req->duration), can_bus::steady_seconds());
        }
        catch (const std::exception & e) {res->message = e.what();}
        if (!res->success && res->message.empty()) {res->message = "bus not fresh and quiet or maintenance busy";}
      });
    try {
      if (backend_ == "serial") {
        if (router_.allow_fd) {throw std::invalid_argument("legacy serial adapter does not support CAN-FD");}
        transport_ = can_bus::make_serial_transport(port);
      } else if (backend_ == "socketcan") {
        transport_ = can_bus::make_socketcan_transport(port, router_.allow_fd);
      } else if (backend_ != "sim") {throw std::invalid_argument("unknown CAN backend");}
      ready_ = true;
    } catch (const std::exception & e) {error_ = e.what();}
    io_timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate)),
      [this] {tick();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {publish_status();});
    publish_status();
    observer_running_ = true;
    observer_thread_ = std::thread([this] {publish_observations();});
  }
  ~BusNode() override
  {
    observer_running_ = false;
    observer_condition_.notify_all();
    router_.shutdown();
    for (const auto & f : router_.drain(can_bus::steady_seconds())) {
      try {transmit(f);} catch (...) {}
    }
    if (observer_thread_.joinable()) {observer_thread_.join();}
  }
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<BusNode>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("can_bus"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
