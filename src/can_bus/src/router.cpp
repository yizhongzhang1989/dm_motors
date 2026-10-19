#include "can_bus/router.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace can_bus
{
namespace
{
void validate_filter(const Filter & f)
{
  const uint32_t max = f.extended ? 0x1fffffff : 0x7ff;
  if (f.id > max || f.mask > max || f.channel || f.prefix.size() > 64 ||
    f.prefix.size() != f.prefix_mask.size())
  {throw std::invalid_argument("invalid frame filter");}
}
bool same(const Frame & a, const Frame & b)
{
  return a.id == b.id && a.length == b.length && a.channel == b.channel &&
    a.fd == b.fd && a.brs == b.brs && a.extended == b.extended && a.rtr == b.rtr &&
    std::equal(a.data.begin(), a.data.begin() + a.length, b.data.begin());
}
bool allowed(const Client & c, const Frame & f)
{
  return std::any_of(c.tx.begin(), c.tx.end(), [&](const auto & filter) {return matches(filter, f);});
}
bool polling(const Client & c, const Frame & f)
{
  return std::any_of(c.polls.begin(), c.polls.end(), [&](const auto & poll) {return same(poll, f);});
}
bool expired(const Client & c, double now)
{
  // Discovery reserves routes, not motion authority; the first heartbeat activates the lease.
  return now - c.last_heartbeat > (c.heartbeat_sequence ? c.lease : 2.0);
}
}
bool matches(const Filter & filter, const Frame & f)
{
  if (filter.extended != f.extended || filter.channel != f.channel ||
    ((filter.id ^ f.id) & filter.mask) || f.length < filter.prefix.size()) {return false;}
  for (size_t i = 0; i < filter.prefix.size(); ++i) {
    if ((filter.prefix[i] ^ f.data[i]) & filter.prefix_mask[i]) {return false;}
  }
  return true;
}
bool overlaps(const Filter & a, const Filter & b)
{
  if (a.extended != b.extended || a.channel != b.channel ||
    ((a.id ^ b.id) & a.mask & b.mask)) {return false;}
  for (size_t i = 0; i < std::min(a.prefix.size(), b.prefix.size()); ++i) {
    if ((a.prefix[i] ^ b.prefix[i]) & a.prefix_mask[i] & b.prefix_mask[i]) {return false;}
  }
  return true;
}
Client & Router::authenticate(const std::string & token, uint64_t requested_epoch)
{
  auto found = clients.find(token);
  if (requested_epoch != epoch || found == clients.end() || found->second.revoked) {
    throw std::invalid_argument("stale epoch or revoked/unknown client token");
  }
  return found->second;
}
std::string Router::add(Client c, double now)
{
  const auto previous = std::find_if(clients.begin(), clients.end(),
    [&](const auto & item) {return item.second.name == c.name && item.second.revoked;});
  if ((clients.size() >= 128 && previous == clients.end()) ||
    c.name.empty() || c.name.size() > 96 || c.group.size() > 96 ||
    c.rx.size() > 32 || c.tx.size() > 32 || c.stops.size() > 16 || c.polls.size() > 16 ||
    !std::isfinite(c.lease) || c.lease < .3 || c.lease > 1 ||
    !std::isfinite(c.max_tx_hz) || c.max_tx_hz <= 0 || c.max_tx_hz > 10000)
  {throw std::invalid_argument("invalid registration or bus client capacity reached");}
  for (const auto & f : c.rx) {validate_filter(f);}
  for (const auto & f : c.tx) {validate_filter(f);}
  for (const auto & item : clients) {
    if (item.second.name == c.name && item.second.revoked) {continue;}
    if (item.second.name == c.name) {throw std::invalid_argument("client name already registered");}
    for (const auto & a : c.tx) {
      for (const auto & b : item.second.tx) {
        if (overlaps(a, b)) {throw std::invalid_argument("transmit filter ownership overlaps");}
      }
    }
  }
  for (const auto & f : c.stops) {
    validate_frame(f, allow_fd);
    if (!allowed(c, f)) {throw std::invalid_argument("stop frame outside transmit ownership");}
  }
  for (const auto & f : c.polls) {
    validate_frame(f, allow_fd);
    if (!allowed(c, f)) {throw std::invalid_argument("poll frame outside transmit ownership");}
  }
  c.token = "c" + std::to_string(epoch) + "_" + std::to_string(++next_token_);
  c.last_heartbeat = now;
  auto token = c.token;
  if (previous != clients.end()) {clients.erase(previous);}
  clients.emplace(token, std::move(c));
  return token;
}
void Router::revoke(Client & c)
{
  if (c.revoked) {return;}
  c.revoked = true; c.pending.clear();
  for (const auto & f : c.stops) {emergency_.push_back(f);}
  if (!c.group.empty()) {group_faults.insert(c.group);}
  if (maintenance_owner == c.token) {maintenance_owner.clear();}
}
void Router::heartbeat(const std::string & token, uint64_t e, uint64_t seq,
  bool quiet, bool fault, double now, uint64_t sent_steady)
{
  auto & c = authenticate(token, e);
  if (expired(c, now)) {revoke(c); throw std::invalid_argument("lease expired");}
  if (seq <= c.heartbeat_sequence) {throw std::invalid_argument("nonmonotonic heartbeat");}
  c.heartbeat_sequence = seq; c.last_heartbeat = now;
  if (!sent_steady) {sent_steady = static_cast<uint64_t>(now * 1e9);}
  const bool pending_active = std::any_of(c.pending.begin(), c.pending.end(),
    [](const auto & p) {return p.active;});
  const bool stop_confirmable = !c.activity_requires_stop ||
    (c.stop_dispatched_steady && sent_steady >= c.stop_dispatched_steady);
  if (!quiet || (sent_steady >= c.last_activity_steady && !pending_active && stop_confirmable)) {
    c.quiet = quiet;
    if (quiet) {c.activity_requires_stop = false;}
  }
  if (fault && !c.fault) {
    c.pending.clear();
    if (c.group.empty()) {
      for (const auto & f : c.stops) {emergency_.push_back(f);}
    } else {
      revoke(c);
    }
  }
  c.fault = fault;
}
void Router::enqueue(const std::string & token, uint64_t e, uint64_t seq,
  double ttl, uint8_t priority, const Frame & f, double now, bool active, uint64_t sent_steady)
{
  auto & c = authenticate(token, e);
  if (expired(c, now)) {revoke(c); throw std::invalid_argument("lease expired");}
  validate_frame(f, allow_fd);
  if (seq <= c.tx_sequence || !std::isfinite(ttl) || ttl <= 0 || ttl > .25 ||
    priority > 2 || !allowed(c, f)) {throw std::invalid_argument("invalid TX sequence, TTL, priority or route");}
  if (priority == 1) {
    if (active) {throw std::invalid_argument("stop cannot mark activity");}
    if (!std::any_of(c.stops.begin(), c.stops.end(), [&](const auto & s) {return same(s, f);})) {
      throw std::invalid_argument("priority stop must equal a registered stop frame");
    }
    c.pending.erase(std::remove_if(c.pending.begin(), c.pending.end(),
      [&](const auto & p) {return p.priority != 1 || same(p.frame, f);}), c.pending.end());
  } else {
    const bool maintenance_poll = priority == 0 && !active && polling(c, f);
    if ((!c.group.empty() && group_faults.count(c.group)) ||
      (!maintenance_owner.empty() && maintenance_owner != token && !maintenance_poll) ||
      (priority == 2 && maintenance_owner != token) ||
      (active && (!maintenance_owner.empty() || c.fault)))
    {throw std::invalid_argument("fault group latched or bus maintenance locked");}
    // Coalesce only within an identical registered route, including payload prefix.
    if (priority == 0) {
      auto route = std::find_if(c.tx.begin(), c.tx.end(), [&](const auto & r) {return matches(r, f);});
      for (auto it = c.pending.begin(); it != c.pending.end();) {
        if (it->priority == priority && it->frame.id == f.id && matches(*route, it->frame)) {
          it = c.pending.erase(it);
        } else {++it;}
      }
    }
  }
  if (c.pending.size() >= 32) {throw std::invalid_argument("client transmit queue full");}
  c.tx_sequence = seq;
  if (active) {
    c.quiet = false;
    c.activity_requires_stop = true; c.stop_dispatched_steady = 0;
    c.last_activity_steady = sent_steady ? sent_steady : static_cast<uint64_t>(now * 1e9);
  }
  c.pending.push_back({f, now + ttl, priority, active});
}
bool Router::maintenance(const std::string & token, uint64_t e,
  bool acquire, double duration, double now)
{
  auto & owner = authenticate(token, e);
  if (expired(owner, now)) {revoke(owner); return false;}
  if (!acquire) {
    if (maintenance_owner == token) {maintenance_owner.clear();}
    return true;
  }
  if (!std::isfinite(duration) || duration <= 0 || duration > 1) {return false;}
  if (!maintenance_owner.empty() && maintenance_owner != token) {return false;}
  for (const auto & item : clients) {
    const auto & c = item.second;
    if (!c.revoked && (!c.heartbeat_sequence || !c.quiet || expired(c, now))) {return false;}
  }
  if (maintenance_owner.empty()) {
    for (auto & item : clients) {
      auto & queue = item.second.pending;
      queue.erase(std::remove_if(queue.begin(), queue.end(),
        [](const auto & p) {return p.priority != 1;}), queue.end());
    }
  }
  maintenance_owner = token; maintenance_deadline_ = now + duration;
  return true;
}
std::vector<Frame> Router::drain(double now, bool transmit_ready)
{
  if (!maintenance_owner.empty() && now > maintenance_deadline_) {
    revoke(clients.at(maintenance_owner));
    maintenance_owner.clear();
  }
  for (auto & item : clients) {
    auto & c = item.second;
    if (!c.revoked && expired(c, now)) {revoke(c);}
  }
  // Group failure independently stops every group member, even before status delivery.
  for (auto & item : clients) {
    auto & c = item.second;
    if (!c.revoked && !c.group.empty() && group_faults.count(c.group)) {revoke(c);}
  }
  std::vector<Frame> result;
  if (transmit_ready) {
    result.assign(emergency_.begin(), emergency_.end()); emergency_.clear();
    for (const auto & frame : result) {
      for (auto & item : clients) {
        auto & c = item.second;
        if (c.activity_requires_stop && std::any_of(c.stops.begin(), c.stops.end(),
          [&](const auto & registered) {return same(registered, frame);}))
        {c.stop_dispatched_steady = static_cast<uint64_t>(now * 1e9);}
      }
    }
  }
  size_t priority_end = result.size();
  for (auto & item : clients) {
    auto & c = item.second;
    for (auto it = c.pending.begin(); it != c.pending.end();) {
      if (now > it->expires) {it = c.pending.erase(it); ++rejected;}
      else {++it;}
    }
    if (c.pending.empty() || c.revoked || !transmit_ready) {continue;}
    auto stop = std::find_if(c.pending.begin(), c.pending.end(), [](const auto & p) {return p.priority == 1;});
    if (stop != c.pending.end()) {
      if (result.capacity() == 0) {result.reserve(clients.size());}
      result.insert(result.begin() + priority_end, stop->frame); ++priority_end;
      c.pending.erase(stop);
      if (c.activity_requires_stop) {c.stop_dispatched_steady = static_cast<uint64_t>(now * 1e9);}
      continue;
    }
    if (!c.heartbeat_sequence || !c.receive_ready) {continue;}
    const auto & pending = c.pending.front();
    const bool maintenance_poll = pending.priority == 0 && !pending.active && polling(c, pending.frame);
    if ((!maintenance_owner.empty() && maintenance_owner != c.token && !maintenance_poll) ||
      now < c.next_tx) {continue;}
    const auto frame = c.pending.front().frame;
    if (result.capacity() == 0) {result.reserve(clients.size());}
    result.push_back(frame); c.pending.pop_front();
    if (c.activity_requires_stop && std::any_of(c.stops.begin(), c.stops.end(),
      [&](const auto & registered) {return same(registered, frame);}))
    {c.stop_dispatched_steady = static_cast<uint64_t>(now * 1e9);}
    c.next_tx = now + 1.0 / c.max_tx_hz;
  }
  return result;
}
void Router::shutdown() {for (auto & item : clients) {revoke(item.second);}}
size_t Router::queue_depth() const
{
  size_t n = emergency_.size();
  for (const auto & item : clients) {n += item.second.pending.size();}
  return n;
}
}
