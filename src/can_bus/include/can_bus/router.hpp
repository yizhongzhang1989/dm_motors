#pragma once
#include "can_bus/transport.hpp"
#include <deque>
#include <map>
#include <set>
namespace can_bus
{
struct Filter
{
  uint32_t id{0}, mask{0};
  bool extended{false};
  uint8_t channel{0};
  std::vector<uint8_t> prefix, prefix_mask;
};
bool matches(const Filter &, const Frame &);
bool overlaps(const Filter &, const Filter &);
struct Client
{
  std::string name, token, group;
  std::vector<Filter> rx, tx;
  std::vector<Frame> stops;
  double lease{0.5}, last_heartbeat{0}, max_tx_hz{1000}, next_tx{0};
  uint64_t tx_sequence{0}, heartbeat_sequence{0};
  uint64_t last_activity_steady{0};
  uint64_t stop_dispatched_steady{0};
  bool activity_requires_stop{false};
  bool receive_ready{true};
  bool quiet{true}, revoked{false}, fault{false};
  struct Pending {Frame frame; double expires; uint8_t priority; bool active;};
  std::deque<Pending> pending;
};
class Router
{
public:
  explicit Router(uint64_t epoch, bool fd = false) : epoch(epoch), allow_fd(fd) {}
  std::string add(Client client, double now);
  void heartbeat(const std::string &, uint64_t, uint64_t, bool, bool, double, uint64_t sent_steady = 0);
  void enqueue(const std::string &, uint64_t, uint64_t, double, uint8_t,
    const Frame &, double, bool active = false, uint64_t sent_steady = 0);
  bool maintenance(const std::string &, uint64_t, bool, double, double);
  std::vector<Frame> drain(double now, bool transmit_ready = true);
  void shutdown();
  size_t queue_depth() const;
  uint64_t epoch, rejected{0};
  bool allow_fd;
  std::map<std::string, Client> clients;
  std::set<std::string> group_faults;
  std::string maintenance_owner;
private:
  Client & authenticate(const std::string &, uint64_t);
  void revoke(Client &);
  double maintenance_deadline_{0};
  uint64_t next_token_{0};
  std::deque<Frame> emergency_;
};
}
