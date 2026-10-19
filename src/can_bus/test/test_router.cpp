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
TEST(Router, HeartbeatExpiryStopsAndLatchesGroup)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  const auto b = router.add(client("b", 4), 0);
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
  router.heartbeat(b, 42, 1, false, false, .01);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .02));
  router.heartbeat(b, 42, 2, true, false, .03);
  EXPECT_TRUE(router.maintenance(a, 42, true, .6, .04));
  can_bus::Frame f; f.id = 4;
  EXPECT_THROW(router.enqueue(b, 42, 1, .1, 0, f, .05), std::invalid_argument);
  EXPECT_FALSE(router.maintenance(b, 42, true, .6, .05));
  EXPECT_TRUE(router.maintenance(a, 42, false, .6, .06));
}
TEST(Router, ActivityAndStaleQuietHeartbeatCannotRaceMaintenance)
{
  can_bus::Router router(42);
  const auto a = router.add(client("a", 3), 0);
  const auto b = router.add(client("b", 4), 0);
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
  can_bus::Frame command; command.id = 4;
  router.enqueue(b, 42, 1, .1, 0, command, .01, true, 10000000);
  router.drain(.02);
  router.heartbeat(b, 42, 1, true, false, .03, 30000000);
  EXPECT_FALSE(router.clients.at(b).quiet);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .04));
  router.enqueue(b, 42, 2, .1, 1, device.stops.front(), .05);
  router.heartbeat(b, 42, 2, true, false, .06, 60000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .07));
  router.drain(.08);
  router.heartbeat(b, 42, 3, true, false, .09, 60000000);
  EXPECT_FALSE(router.maintenance(a, 42, true, .6, .10));
  router.heartbeat(b, 42, 4, true, false, .11, 110000000);
  EXPECT_TRUE(router.clients.at(b).quiet);
  EXPECT_TRUE(router.maintenance(a, 42, true, .6, .12));
}
TEST(Router, MaintenanceFifoHasBoundedCapacity)
{
  can_bus::Router router(42);
  const auto token = router.add(client("device", 3), 0);
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
