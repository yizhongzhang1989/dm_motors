#include "can_bus/router.hpp"
#include <gtest/gtest.h>
#include <stdexcept>
namespace
{
can_bus::Client client(const std::string & name, uint8_t id)
{
  can_bus::Client c; c.name = name; c.group = "arm";
  c.tx = {{id, 0x7ff, false, 0, {}, {}},
    {0x7ff, 0x7ff, false, 0, {id, 0}, {255, 255}}};
  can_bus::Frame f; f.id = id; f.data.fill(255); f.data[7] = 0xfd;
  c.stops = {f}; return c;
}
can_bus::Frame position_read(uint8_t id)
{
  can_bus::Frame f; f.id = 0x7ff;
  f.data[0] = id; f.data[2] = 0x33; f.data[3] = 80;
  return f;
}
TEST(Router, OptionalPositionReadHasNarrowClassicInactiveWhitelist)
{
  can_bus::Router router(42, true);  // An FD-capable bus may still carry classical frames.
  const auto token = router.add(client("device", 3), 0);
  const auto good = position_read(3);
  EXPECT_NO_THROW(router.enqueue(token, 42, 1, .02, 3, good, .01));
  EXPECT_THROW(router.enqueue(token, 42, 2, .02, 3, good, .01, true), std::invalid_argument);
  std::vector<can_bus::Frame> bad;
  auto f = good; f.data[3] = 81; bad.push_back(f);
  f = good; f.data[2] = 0x55; bad.push_back(f);
  f = good; f.data[0] = 4; bad.push_back(f);
  f = good; f.data[1] = 1; bad.push_back(f);
  for (size_t i = 4; i < 8; ++i) {f = good; f.data[i] = 1; bad.push_back(f);}
  f = good; f.length = 7; bad.push_back(f);
  f = good; f.fd = true; bad.push_back(f);
  f = good; f.brs = true; bad.push_back(f);
  f = good; f.extended = true; bad.push_back(f);
  f = good; f.rtr = true; bad.push_back(f);
  f = good; f.error = true; bad.push_back(f);
  f = good; f.channel = 1; bad.push_back(f);
  f = good; f.id = 3; bad.push_back(f);
  for (const auto & frame : bad) {
    EXPECT_THROW(router.enqueue(token, 42, 2, .02, 3, frame, .01), std::invalid_argument);
  }
  EXPECT_EQ(router.clients.at(token).tx_sequence, 1u);
  EXPECT_EQ(router.queue_depth(), 1u);
}
TEST(Router, OptionalReadNeedsAnExplicitNonzeroOwnedMotorPrefix)
{
  for (unsigned variant = 0; variant < 4; ++variant) {
    can_bus::Router router(42);
    auto c = client("device", variant == 3 ? 0 : 3);
    if (variant == 0) {c.tx[1].prefix.clear(); c.tx[1].prefix_mask.clear();}
    if (variant == 1) {c.tx[1].prefix_mask[1] = 0;}
    if (variant == 2) {c.tx[1].mask = 0x700;}
    const auto token = router.add(c, 0);
    EXPECT_THROW(router.enqueue(token, 42, 1, .02, 3,
      position_read(variant == 3 ? 0 : 3), .01), std::invalid_argument);
  }
}
TEST(Router, OptionalDuplicateRejectsWithoutReplacementOrActivityMutation)
{
  can_bus::Router router(42);
  const auto token = router.add(client("device", 3), 0);
  router.heartbeat(token, 42, 1, true, false, 0);
  const auto read = position_read(3);
  router.enqueue(token, 42, 1, .02, 3, read, .01);
  EXPECT_THROW(router.enqueue(token, 42, 2, .02, 3, read, .015), std::invalid_argument);
  const auto & c = router.clients.at(token);
  EXPECT_EQ(c.tx_sequence, 1u); ASSERT_EQ(c.pending.size(), 1u);
  EXPECT_DOUBLE_EQ(c.pending.front().expires, .03);
  EXPECT_TRUE(c.quiet); EXPECT_FALSE(c.activity_requires_stop); EXPECT_EQ(c.last_activity_steady, 0u);
  ASSERT_EQ(router.drain(.02).size(), 1u);
  EXPECT_TRUE(c.quiet); EXPECT_FALSE(c.activity_requires_stop);
  can_bus::Frame command; command.id = 3;
  router.enqueue(token, 42, 2, .1, 0, command, .03, true, 30000000);
  ASSERT_EQ(router.drain(.031).size(), 1u);
  router.enqueue(token, 42, 3, .1, 3, read, .032);
  EXPECT_FALSE(c.quiet); EXPECT_TRUE(c.activity_requires_stop);
  EXPECT_EQ(c.last_activity_steady, 30000000u);
  ASSERT_EQ(router.drain(.033).size(), 1u);
  EXPECT_FALSE(c.quiet); EXPECT_TRUE(c.activity_requires_stop);
  EXPECT_EQ(c.last_activity_steady, 30000000u);
}
TEST(Router, OrdinaryCoalescingKeepsOptionalAndOrdinaryRunsFirst)
{
  can_bus::Router router(42);
  const auto token = router.add(client("device", 3), 0);
  router.heartbeat(token, 42, 1, true, false, 0);
  router.enqueue(token, 42, 1, .1, 3, position_read(3), .01);
  auto ordinary = position_read(3); ordinary.data[2] = 0xcc; ordinary.data[3] = 0;
  router.enqueue(token, 42, 2, .1, 0, ordinary, .011);
  router.enqueue(token, 42, 3, .1, 0, ordinary, .012);
  ASSERT_EQ(router.queue_depth(), 2u);
  auto frames = router.drain(.02);
  ASSERT_EQ(frames.size(), 1u); EXPECT_EQ(frames[0].data[2], 0xcc);
  EXPECT_EQ(router.clients.at(token).pending.front().priority, 3);
  EXPECT_TRUE(router.drain(.0205).empty());
  frames = router.drain(.022);
  ASSERT_EQ(frames.size(), 1u); EXPECT_EQ(frames[0].data[2], 0x33);
}
TEST(Router, OtherClientsOrdinaryPrecedesButDoesNotStarveOptional)
{
  can_bus::Router router(42);
  const auto a = router.add(client("optional_first", 3), 0);
  const auto b = router.add(client("ordinary_second", 4), 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  router.heartbeat(b, 42, 1, true, false, 0);
  router.enqueue(a, 42, 1, .1, 3, position_read(3), .01);
  can_bus::Frame ordinary; ordinary.id = 4;
  router.enqueue(b, 42, 1, .1, 0, ordinary, .01);
  const auto frames = router.drain(.02);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id, 4u); EXPECT_EQ(frames[1].id, 0x7ffu);
  EXPECT_EQ(router.queue_depth(), 0u);
}
TEST(Router, StopCancelsOptionalAndQueuedStopRejectsNewOptional)
{
  can_bus::Router router(42);
  const auto c = client("device", 3);
  const auto token = router.add(c, 0);
  router.heartbeat(token, 42, 1, true, false, 0);
  router.enqueue(token, 42, 1, .1, 3, position_read(3), .01);
  can_bus::Frame ordinary; ordinary.id = 3;
  router.enqueue(token, 42, 2, .1, 0, ordinary, .01, true);
  router.enqueue(token, 42, 3, .1, 1, c.stops.front(), .02);
  EXPECT_THROW(router.enqueue(token, 42, 4, .1, 3, position_read(3), .021), std::invalid_argument);
  ASSERT_EQ(router.queue_depth(), 1u);
  const auto frames = router.drain(.03);
  ASSERT_EQ(frames.size(), 1u); EXPECT_EQ(frames[0].data[7], 0xfd);
  EXPECT_TRUE(router.drain(.04).empty());
}
TEST(Router, MixedClientsKeepStopOrdinaryOptionalOrder)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  const auto b = router.add(client("b", 4), 0);
  const auto c = router.add(client("c", 5), 0);
  for (const auto & token : {a, b, c}) {router.heartbeat(token, 42, 1, true, false, 0);}
  router.enqueue(a, 42, 1, .1, 3, position_read(3), .01);
  can_bus::Frame normal; normal.id = 4;
  router.enqueue(b, 42, 1, .1, 0, normal, .01);
  router.enqueue(c, 42, 1, .1, 1, client("c", 5).stops.front(), .01);
  const auto frames = router.drain(.02);
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0].id, 5u); EXPECT_EQ(frames[1].id, 4u); EXPECT_EQ(frames[2].id, 0x7ffu);
}
TEST(Router, OptionalRespectsMaintenanceFaultReadinessAndExpiry)
{
  can_bus::Router router(42);
  auto c = client("device", 3); c.group.clear();
  const auto token = router.add(c, 0);
  router.heartbeat(token, 42, 1, true, false, 0);
  router.enqueue(token, 42, 1, .01, 3, position_read(3), .01);
  router.clients.at(token).receive_ready = false;
  EXPECT_TRUE(router.drain(.015).empty());
  EXPECT_TRUE(router.drain(.025).empty()); EXPECT_EQ(router.rejected, 1u);
  router.clients.at(token).receive_ready = true;
  router.enqueue(token, 42, 2, .1, 3, position_read(3), .03);
  ASSERT_TRUE(router.maintenance(token, 42, true, .2, .04));
  EXPECT_EQ(router.queue_depth(), 0u);
  EXPECT_THROW(router.enqueue(token, 42, 3, .1, 3, position_read(3), .05), std::invalid_argument);
  EXPECT_TRUE(router.maintenance(token, 42, false, .2, .06));
  router.heartbeat(token, 42, 2, true, true, .07);
  EXPECT_THROW(router.enqueue(token, 42, 3, .1, 3, position_read(3), .08), std::invalid_argument);
}
TEST(Router, GenericOwnershipAndObservers)
{
  can_bus::Router router(42);
  auto a = client("other_device", 3); a.rx = {{0x123, 0x7ff, false, 0, {}, {}}};
  const auto token = router.add(a, 0);
  auto b = client("another_device", 4); b.rx = a.rx;
  EXPECT_NO_THROW(router.add(b, 0));
  EXPECT_THROW(router.add(client("conflicting_device", 3), 0), std::invalid_argument);
  can_bus::Frame f; f.id = 0x7ff; f.data[0] = 4;
  EXPECT_THROW(router.enqueue(token, 42, 1, .1, 0, f, .01), std::invalid_argument);
  f.data[0] = 3;
  EXPECT_NO_THROW(router.enqueue(token, 42, 1, .1, 0, f, .01));
}
TEST(Router, EpochSequenceTtlAndBudget)
{
  can_bus::Router router(42);
  const auto token = router.add(client("device", 3), 0);
  can_bus::Frame f; f.id = 3;
  EXPECT_THROW(router.enqueue(token, 41, 1, .1, 0, f, 0), std::invalid_argument);
  EXPECT_THROW(router.enqueue(token, 42, 1, 0, 0, f, 0), std::invalid_argument);
  for (unsigned i = 1; i <= 100; ++i) {router.enqueue(token, 42, i, .1, 0, f, 0);}
  EXPECT_EQ(router.queue_depth(), 1u);
  EXPECT_THROW(router.enqueue(token, 42, 99, .1, 0, f, 0), std::invalid_argument);
  EXPECT_TRUE(router.drain(.2).empty());
  EXPECT_EQ(router.rejected, 1u);
}
TEST(Router, RegistrationNeedsFreshHeartbeatBeforeNormalTrafficOrMaintenance)
{
  can_bus::Router router(42);
  auto c = client("discovering", 3); c.group.clear();
  const auto token = router.add(c, 0);
  can_bus::Frame command; command.id = 3;
  router.enqueue(token, 42, 1, .1, 0, command, .01, true, 10000000);
  EXPECT_TRUE(router.drain(.02).empty());
  EXPECT_FALSE(router.maintenance(token, 42, true, .6, .03));
  EXPECT_TRUE(router.drain(.2).empty());
  EXPECT_EQ(router.rejected, 1u);
  EXPECT_TRUE(router.drain(.6).empty());
  EXPECT_FALSE(router.clients.at(token).revoked);
  ASSERT_NO_THROW(router.heartbeat(token, 42, 1, false, false, .7, 700000000));
  router.enqueue(token, 42, 2, .1, 0, command, .71, true, 710000000);
  EXPECT_EQ(router.drain(.72).size(), 1u);
  EXPECT_EQ(router.drain(1.21).size(), 1u);
  EXPECT_TRUE(router.clients.at(token).revoked);
}
TEST(Router, UnactivatedRegistrationIsBoundedAndStopsStillBypassTheHandshake)
{
  can_bus::Router router(42);
  auto c = client("missing_heartbeat", 3); c.group.clear();
  const auto token = router.add(c, 0);
  router.enqueue(token, 42, 1, .1, 1, c.stops.front(), .01);
  ASSERT_EQ(router.drain(.02).size(), 1u);
  EXPECT_TRUE(router.drain(1.9).empty());
  EXPECT_FALSE(router.clients.at(token).revoked);
  EXPECT_EQ(router.drain(2.01).size(), 1u);
  EXPECT_TRUE(router.clients.at(token).revoked);
  EXPECT_THROW(router.heartbeat(token, 42, 1, true, false, 2.02), std::invalid_argument);
}
TEST(Router, MaintenanceWaitsForEveryClientHeartbeatHandshake)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .01));
  router.heartbeat(a, 42, 1, true, false, .02);
  const auto b = router.add(client("b", 4), .02);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .03));
  router.heartbeat(b, 42, 1, true, false, .04);
  EXPECT_TRUE(router.maintenance(a, 42, true, .6, .05));
}
TEST(Router, HeartbeatExpiryStopsAndLatchesGroup)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  const auto b = router.add(client("b", 4), 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  router.heartbeat(b, 42, 1, true, false, .3);
  const auto frames = router.drain(.51);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_TRUE(router.clients.at(a).revoked);
  EXPECT_TRUE(router.clients.at(b).revoked);
  EXPECT_EQ(router.group_faults.count("arm"), 1u);
  EXPECT_THROW(router.heartbeat(a, 42, 1, true, false, .52), std::invalid_argument);
}
TEST(Router, ExplicitRelaunchGetsNewTokenAndCannotReviveOldEpochLease)
{
  can_bus::Router router(42);
  auto c = client("device", 3); c.group.clear();
  const auto first = router.add(c, 0);
  router.heartbeat(first, 42, 1, true, false, 0);
  router.drain(.6);
  const auto second = router.add(c, .7);
  EXPECT_NE(first, second);
  EXPECT_EQ(router.clients.size(), 1u);
  EXPECT_THROW(router.heartbeat(first, 42, 2, true, false, .71), std::invalid_argument);
  EXPECT_NO_THROW(router.heartbeat(second, 42, 1, true, false, .71));
}
TEST(Router, UngroupedFaultStopsButRetainsPollingAndExplicitReset)
{
  can_bus::Router router(42);
  auto c = client("device", 3); c.group.clear();
  const auto token = router.add(c, 0);
  router.heartbeat(token, 42, 1, true, true, .01);
  const auto stops = router.drain(.02);
  ASSERT_EQ(stops.size(), 1u);
  EXPECT_FALSE(router.clients.at(token).revoked);
  can_bus::Frame poll; poll.id = 0x7ff; poll.data[0] = 3;
  EXPECT_NO_THROW(router.enqueue(token, 42, 1, .1, 0, poll, .03));
  EXPECT_EQ(router.drain(.04).size(), 1u);
  can_bus::Frame command; command.id = 3;
  EXPECT_THROW(router.enqueue(token, 42, 2, .1, 0, command, .05, true), std::invalid_argument);
  router.heartbeat(token, 42, 2, true, false, .06);
  EXPECT_NO_THROW(router.enqueue(token, 42, 2, .1, 0, command, .07, true));
}
TEST(Router, StopPriorityIsExactAndClearsOldNormal)
{
  can_bus::Router router(42);
  auto c = client("device", 3);
  const auto token = router.add(c, 0);
  can_bus::Frame f; f.id = 3;
  router.enqueue(token, 42, 1, .1, 0, f, 0);
  EXPECT_THROW(router.enqueue(token, 42, 2, .1, 1, f, 0), std::invalid_argument);
  router.enqueue(token, 42, 3, .1, 1, c.stops.front(), 0);
  auto frames = router.drain(.01);
  ASSERT_EQ(frames.size(), 1u); EXPECT_EQ(frames[0].data[7], 0xfd);
}
TEST(Router, MaintenanceRequiresFreshQuietClients)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  const auto b = router.add(client("b", 4), 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  router.heartbeat(b, 42, 1, false, false, .01);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .02));
  router.heartbeat(b, 42, 2, true, false, .03);
  EXPECT_TRUE(router.maintenance(a, 42, true, .6, .04));
  can_bus::Frame f; f.id = 4;
  EXPECT_THROW(router.enqueue(b, 42, 1, .1, 0, f, .05), std::invalid_argument);
  EXPECT_FALSE(router.maintenance(b, 42, true, .6, .05));
  EXPECT_TRUE(router.maintenance(a, 42, false, .6, .06));
}
TEST(Router, MaintenanceAllowsOnlyExactDeclaredInactivePollsFromOtherClients)
{
  can_bus::Router router(42);
  const auto a = router.add(client("owner", 3), 0);
  auto device = client("polling_device", 4);
  can_bus::Frame poll; poll.id = 4; poll.data[0] = 0x42;
  device.polls = {poll};
  const auto b = router.add(device, 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  router.heartbeat(b, 42, 1, true, false, 0);
  ASSERT_TRUE(router.maintenance(a, 42, true, .6, .01));
  EXPECT_NO_THROW(router.enqueue(b, 42, 1, .1, 0, poll, .02));
  const auto sent = router.drain(.03);
  ASSERT_EQ(sent.size(), 1u);
  EXPECT_EQ(sent.front().id, 4u);
  EXPECT_EQ(sent.front().data, poll.data);
  EXPECT_TRUE(router.clients.at(b).quiet);
  EXPECT_EQ(router.maintenance_owner, a);
  auto changed = poll; changed.data[0] ^= 1;
  EXPECT_THROW(router.enqueue(b, 42, 2, .1, 0, changed, .04), std::invalid_argument);
  EXPECT_THROW(router.enqueue(b, 42, 2, .1, 0, poll, .04, true), std::invalid_argument);
  EXPECT_THROW(router.enqueue(b, 42, 2, .1, 2, poll, .04), std::invalid_argument);
}
TEST(Router, DeclaredPollsMustBeValidBoundedAndWithinOwnedRoutes)
{
  can_bus::Router router(42);
  auto c = client("invalid_poll", 3);
  can_bus::Frame poll; poll.id = 4;
  c.polls = {poll};
  EXPECT_THROW(router.add(c, 0), std::invalid_argument);
  poll.id = 3; poll.length = 9;
  c.polls = {poll};
  EXPECT_THROW(router.add(c, 0), std::invalid_argument);
  poll.length = 8;
  c.polls.assign(17, poll);
  EXPECT_THROW(router.add(c, 0), std::invalid_argument);
}
TEST(Router, ActivityAndStaleQuietHeartbeatCannotRaceMaintenance)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  const auto b = router.add(client("b", 4), 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  can_bus::Frame f; f.id = 4;
  router.enqueue(b, 42, 1, .1, 0, f, .02, true, 20000000);
  router.heartbeat(b, 42, 1, true, false, .03, 10000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .04));
  router.drain(.05);
  router.heartbeat(b, 42, 2, true, false, .06, 10000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .07));
  router.heartbeat(b, 42, 3, true, false, .08, 80000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .09));
  router.enqueue(b, 42, 2, .1, 1, client("b", 4).stops.front(), .10);
  router.drain(.11);
  router.heartbeat(b, 42, 4, true, false, .12, 120000000);
  EXPECT_TRUE(router.maintenance(a, 42, true, .6, .13));
  f.id = 3;
  EXPECT_THROW(router.enqueue(a, 42, 1, .1, 0, f, .14, true, 140000000),
    std::invalid_argument);
}
TEST(Router, FreshQuietHeartbeatRequiresDispatchedStopAfterActivity)
{
  can_bus::Router router(42);
  const auto a = router.add(client("observer", 3), 0);
  const auto device = client("active_device", 4);
  const auto b = router.add(device, 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  router.heartbeat(b, 42, 1, true, false, 0);
  can_bus::Frame command; command.id = 4;
  router.enqueue(b, 42, 1, .1, 0, command, .01, true, 10000000);
  router.drain(.02);
  router.heartbeat(b, 42, 2, true, false, .03, 30000000);
  EXPECT_FALSE(router.clients.at(b).quiet);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .04));
  router.enqueue(b, 42, 2, .1, 1, device.stops.front(), .05);
  router.heartbeat(b, 42, 3, true, false, .06, 60000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .07));
  router.drain(.08);
  router.heartbeat(b, 42, 4, true, false, .09, 60000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .10));
  router.heartbeat(b, 42, 5, true, false, .11, 110000000);
  EXPECT_TRUE(router.clients.at(b).quiet);
  EXPECT_TRUE(router.maintenance(a, 42, true, .6, .12));
}
TEST(Router, MaintenanceFifoHasBoundedCapacity)
{
  can_bus::Router router(42);
  const auto token = router.add(client("device", 3), 0);
  router.heartbeat(token, 42, 1, true, false, 0);
  ASSERT_TRUE(router.maintenance(token, 42, true, .6, .01));
  can_bus::Frame f; f.id = 0x7ff; f.data[0] = 3;
  for (unsigned i = 1; i <= 32; ++i) {
    f.data[3] = i;
    router.enqueue(token, 42, i, .1, 2, f, .02);
  }
  EXPECT_EQ(router.queue_depth(), 32u);
  EXPECT_THROW(router.enqueue(token, 42, 33, .1, 2, f, .02), std::invalid_argument);
  auto frames = router.drain(.03);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames.front().data[3], 1);
}
TEST(Router, EndpointReadinessHoldsTrafficWithoutPausingTtlOrLease)
{
  can_bus::Router router(42);
  auto c = client("device", 3); c.group.clear();
  const auto token = router.add(c, 0);
  can_bus::Frame command; command.id = 3;
  router.enqueue(token, 42, 1, .1, 0, command, .01, true, 10000000);
  EXPECT_TRUE(router.drain(.02, false).empty());
  EXPECT_EQ(router.queue_depth(), 1u);
  router.enqueue(token, 42, 2, .1, 1, c.stops.front(), .03);
  EXPECT_TRUE(router.drain(.04, false).empty());
  router.heartbeat(token, 42, 1, true, false, .05, 50000000);
  EXPECT_FALSE(router.clients.at(token).quiet);
  EXPECT_EQ(router.drain(.06, true).size(), 1u);
  router.heartbeat(token, 42, 2, true, false, .07, 70000000);
  EXPECT_TRUE(router.clients.at(token).quiet);
  router.enqueue(token, 42, 3, .1, 0, command, .08);
  EXPECT_TRUE(router.drain(.2, false).empty());
  EXPECT_EQ(router.queue_depth(), 0u);
  EXPECT_EQ(router.rejected, 1u);
  EXPECT_TRUE(router.drain(.6, false).empty());
  EXPECT_TRUE(router.clients.at(token).revoked);
  EXPECT_EQ(router.drain(.61, true).size(), 1u);
}
TEST(Router, PrivateReceiveReadinessHoldsOnlyItsOwnNormalTraffic)
{
  can_bus::Router router(42);
  const auto device = client("late_receiver", 3);
  const auto a = router.add(device, 0);
  const auto b = router.add(client("ready_receiver", 4), 0);
  router.heartbeat(a, 42, 1, true, false, 0);
  router.heartbeat(b, 42, 1, true, false, 0);
  router.clients.at(a).receive_ready = false;
  can_bus::Frame command; command.id = 3;
  router.enqueue(a, 42, 1, .1, 0, command, .01);
  command.id = 4;
  router.enqueue(b, 42, 1, .1, 0, command, .01);
  const auto emitted = router.drain(.02);
  ASSERT_EQ(emitted.size(), 1u);
  EXPECT_EQ(emitted.front().id, 4u);
  EXPECT_EQ(router.clients.at(a).pending.size(), 1u);
  router.enqueue(a, 42, 2, .1, 1, device.stops.front(), .03);
  const auto stopped = router.drain(.04);
  ASSERT_EQ(stopped.size(), 1u);
  EXPECT_EQ(stopped.front().id, 3u);
}
TEST(Frames, ExtendedFdAndLegacyLimitations)
{
  can_bus::Frame f; f.extended = true; f.id = 0x1234567; f.fd = true; f.length = 64; f.brs = true;
  EXPECT_NO_THROW(can_bus::validate_frame(f));
  EXPECT_THROW(can_bus::serial_packet(f), std::invalid_argument);
  f.length = 9; EXPECT_THROW(can_bus::validate_frame(f), std::invalid_argument);
  f = {}; f.length = 4;
  EXPECT_EQ(can_bus::legacy_serial_frame(f).length, 8);
  f.rtr = true; EXPECT_THROW(can_bus::legacy_serial_frame(f), std::invalid_argument);
  can_bus::SerialParser parser;
  std::array<uint8_t, 16> bytes{}; bytes[0] = 0xaa; bytes[1] = 0xee; bytes[15] = 0x55;
  EXPECT_THROW(parser.feed(bytes.data(), bytes.size()), std::runtime_error);
  EXPECT_THROW(parser.feed(bytes.data(), 5000), std::runtime_error);
}
TEST(Router, StopsPrecedeOtherClientsNormalTraffic)
{
  can_bus::Router router(42);
  auto a = client("a", 3);
  auto b = client("b", 4);
  const auto first = router.add(a, 0);
  const auto second = router.add(b, 0);
  router.heartbeat(first, 42, 1, true, false, 0);
  router.heartbeat(second, 42, 1, true, false, 0);
  can_bus::Frame normal; normal.id = 3;
  router.enqueue(first, 42, 1, .1, 0, normal, .01);
  router.enqueue(second, 42, 1, .1, 1, b.stops.front(), .01);
  const auto frames = router.drain(.02);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id, 4u);
  EXPECT_EQ(frames[0].data[7], 0xfd);
  EXPECT_EQ(frames[1].id, 3u);
}
TEST(Frames, SerialParserResynchronizesAndPreservesEveryBatchedFrame)
{
  std::vector<uint8_t> bytes(512, 0x7e);
  for (unsigned i = 0; i < 200; ++i) {
    std::array<uint8_t, 16> frame{};
    frame[0] = 0xaa; frame[1] = 0x11; frame[2] = 8;
    frame[3] = 3; frame[7] = static_cast<uint8_t>(i); frame[15] = 0x55;
    bytes.insert(bytes.end(), frame.begin(), frame.end());
    bytes.push_back(0x7e);
  }
  can_bus::SerialParser parser;
  auto frames = parser.feed(bytes.data(), 1001);
  const auto tail = parser.feed(bytes.data() + 1001, bytes.size() - 1001);
  frames.insert(frames.end(), tail.begin(), tail.end());
  ASSERT_EQ(frames.size(), 200u);
  for (unsigned i = 0; i < frames.size(); ++i) {
    EXPECT_EQ(frames[i].id, 3u);
    EXPECT_EQ(frames[i].data[0], i);
    EXPECT_GT(frames[i].received_steady, 0u);
  }
}
}

TEST(Router, StopRemovalWitnessBindsQueuedRequestAndIsOneShot)
{
  can_bus::Router router(42);
  const auto c = client("device", 3); const auto token = router.add(c, 0);
  router.heartbeat(token, 42, 1, true, false, 0);
  const auto frame = position_read(3);
  EXPECT_TRUE(router.enqueue(token, 42, 11, .02, 3, frame, .01, false, 10000000).empty());
  const auto removed = router.enqueue(token, 42, 12, .1, 1, c.stops[0], .011);
  ASSERT_EQ(removed.size(), 1u); const auto & w = removed[0];
  EXPECT_EQ(w.token, token); EXPECT_EQ(w.epoch, 42u); EXPECT_EQ(w.sequence, 11u);
  EXPECT_EQ(w.stop_sequence, 12u); EXPECT_EQ(w.sent_steady, 10000000u);
  EXPECT_EQ(w.removed_steady, 11000000u); EXPECT_EQ(w.frame.id, frame.id);
  EXPECT_EQ(w.frame.data, frame.data);
  EXPECT_TRUE(router.enqueue(token, 42, 13, .1, 1, c.stops[0], .012).empty());
  const auto tx = router.drain(.012); ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(tx[0].data[7], 0xfd); EXPECT_TRUE(router.drain(.013).empty());
}

TEST(Router, StopRemovalNeverClaimsDrainedExpiredOrdinaryOrFaultClearedReads)
{
  for (unsigned variant = 0; variant < 5; ++variant) {
    SCOPED_TRACE(variant); can_bus::Router router(42);
    auto c = client("device", 3); c.group.clear(); const auto token = router.add(c, 0);
    router.heartbeat(token, 42, 1, true, false, 0);
    router.enqueue(token, 42, 1, .02, variant == 2 ? 0 : 3, position_read(3), .01, false, 10000000);
    double stop_time = .011;
    if (variant == 0) {ASSERT_EQ(router.drain(.01).size(), 1u);}  // Already selected, no TX callback needed.
    if (variant == 1) {stop_time = .031;}  // Expired even if drain has not run yet.
    if (variant == 3) {router.heartbeat(token, 42, 2, true, true, .0105);}
    if (variant == 4) {ASSERT_TRUE(router.maintenance(token, 42, true, .1, .0105));}
    EXPECT_TRUE(router.enqueue(token, 42, 2, .1, 1, c.stops[0], stop_time).empty());
  }
}

TEST(Router, InvalidStopCannotEraseOrCertifyOptionalRead)
{
  can_bus::Router router(42); const auto c = client("device", 3); const auto token = router.add(c, 0);
  router.enqueue(token, 42, 1, .02, 3, position_read(3), .01, false, 10000000);
  auto bad = c.stops[0]; bad.data[7] = 0xfc;
  EXPECT_THROW(router.enqueue(token, 42, 2, .1, 1, bad, .011), std::invalid_argument);
  EXPECT_THROW(router.enqueue(token, 41, 2, .1, 1, c.stops[0], .011), std::invalid_argument);
  ASSERT_EQ(router.queue_depth(), 1u); EXPECT_EQ(router.clients.at(token).pending[0].sequence, 1u);
}
