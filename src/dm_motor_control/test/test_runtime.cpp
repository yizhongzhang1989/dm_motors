#include "dm_motor_control/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <set>
#include <gtest/gtest.h>

using namespace dm_motor;
namespace
{
uint32_t float_bits(float value)
{
  uint32_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}
std::vector<MotorConfig> configuration()
{
  MotorConfig a; a.name = "a";
  MotorConfig b; b.name = "b"; b.id = 2;
  return {a, b};
}
class RecordingTransport : public Transport
{
public:
  explicit RecordingTransport(const std::vector<MotorConfig> & c) : sim(c) {}
  bool maintenance_available() const override {return available;}
  bool activity_available() const override {return motion_available;}
  bool maintenance_blocked() const override {return !available;}
  bool prepare_maintenance() override {return reserved;}
  void send(const Frame & f) override
  {
    if (broken) {throw std::runtime_error("injected transport disconnect");}
    sent.push_back(f);
    if (f.id == 0x7ff && f.data[2] == 0x33 && f.data[3] == 80) {
      if (drop_position_read) {return;}
      if (position_raw_override) {
        Frame reply = f;
        reply.id = 0;
        for (unsigned j = 0; j < 4; ++j) {
          reply.data[4 + j] = (*position_raw_override >> (8 * j)) & 255;
        }
        injected.push_back(reply);
        return;
      }
    }
    if (ignore_write && f.id == 0x7ff && f.data[2] == 0x55) {
      Frame echo = f; echo.id = 0; injected.push_back(echo); return;
    }
    sim.send(f);
  }
  std::vector<Frame> receive() override
  {
    if (broken) {throw std::runtime_error("injected transport disconnect");}
    auto result = sim.receive();
    result.insert(result.end(), injected.begin(), injected.end()); injected.clear();
    return result;
  }
  SimBusTransport sim;
  bool ignore_write{false}, broken{false};
  bool available{true}, reserved{true};
  bool motion_available{true};
  bool drop_position_read{false};
  std::optional<uint32_t> position_raw_override;
  std::vector<Frame> sent, injected;
};
class RuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto t = std::make_unique<RecordingTransport>(configuration()); transport = t.get();
    bus = std::make_unique<Bus>(std::move(t), configuration(), true);
  }
  void step(unsigned count = 1)
  {
    for (unsigned i = 0; i < count; ++i) {now += 0.005; bus->tick(now);}
  }
  void ready()
  {
    for (unsigned i = 0; i < 200 && !bus->ready(); ++i) {step();}
    ASSERT_TRUE(bus->ready()) << bus->initialization_error();
    step(3);
  }
  RegisterOperation await(uint64_t id)
  {
    for (unsigned n = 0; n < 150 && bus->operation(id).state == OperationState::Pending; ++n) {step();}
    return bus->operation(id);
  }
  void enable(const std::string & name)
  {
    bus->enable(name, now); step();
    ASSERT_EQ(bus->snapshot(name, now).phase, Phase::Enabled);
  }
  std::unique_ptr<Bus> bus;
  RecordingTransport * transport{};
  double now{0};
};
}

TEST(RuntimeProtocol, CatalogExactlyMatchesPublishedItems)
{
  const auto & c = register_catalog();
  ASSERT_EQ(c.size(), 49u);
  std::vector<unsigned> ids;
  for (const auto & r : c) {ids.push_back(r.rid);}
  EXPECT_EQ(std::set<unsigned>(ids.begin(), ids.end()).size(), 49u);
  for (const auto & r : c) {
    const bool integer = (r.rid >= 7 && r.rid <= 10) ||
      (r.rid >= 13 && r.rid <= 16) || (r.rid >= 35 && r.rid <= 37);
    EXPECT_EQ(r.integer, integer) << unsigned(r.rid);
    EXPECT_EQ(r.writable, r.rid <= 10 || (r.rid >= 21 && r.rid <= 35));
  }
  EXPECT_EQ(c[37].name, "Boot_ver");
  EXPECT_FALSE(c[29].minimum);
  EXPECT_FALSE(c[29].maximum);
  EXPECT_FALSE(c[3].minimum_inclusive);
  EXPECT_FALSE(c[3].maximum_inclusive);
}
TEST(RuntimeProtocol, FourModesUseGoldenBytesAndLengths)
{
  Target t; t.mode = Mode::PositionVelocity; t.position = 1.0; t.velocity_limit = 2.0;
  auto f = target_frame(2, t, {});
  EXPECT_EQ(f.id, 0x102); EXPECT_EQ(f.length, 8);
  EXPECT_EQ(f.data, (std::array<uint8_t, 8>{0, 0, 0x80, 0x3f, 0, 0, 0, 0x40}));
  t = {}; t.mode = Mode::Velocity; t.velocity = -1;
  f = target_frame(2, t, {});
  EXPECT_EQ(f.id, 0x202); EXPECT_EQ(f.length, 4);
  EXPECT_EQ(f.data, (std::array<uint8_t, 8>{0, 0, 0x80, 0xbf, 0, 0, 0, 0}));
  EXPECT_THROW(serial_packet(f), std::invalid_argument);
  t = {}; t.mode = Mode::PositionForce; t.position = 1;
  t.velocity_limit = 0.5; t.current_ratio = 0.1;
  f = target_frame(2, t, {});
  EXPECT_EQ(f.id, 0x302); EXPECT_EQ(f.length, 8);
  EXPECT_EQ(f.data, (std::array<uint8_t, 8>{0, 0, 0x80, 0x3f, 50, 0, 0xe8, 3}));
  EXPECT_EQ(target_frame(2, {}, {}).data, mit_frame(2, {}, {}).data);
  EXPECT_EQ(special_frame(control_id(2, Mode::PositionForce), 0xfd).id, 0x302);
}
TEST(RuntimeProtocol, RejectsInvalidModeSpecificTargets)
{
  Target t; t.mode = Mode::Velocity; t.velocity = 31;
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t.velocity = 0; t.position = 1;
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t = {}; t.mode = Mode::PositionVelocity; t.velocity_limit = -1;
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t.velocity_limit = 1; t.kp = 1;
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t = {}; t.mode = Mode::PositionForce; t.current_ratio = 1.001;
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t.current_ratio = 0; t.velocity_limit = 101;
  EXPECT_THROW(target_frame(1, t, {12.5, 200, 10}), std::invalid_argument);
  t = {}; t.mode = Mode(5);
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t = {}; t.current_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
  t = {}; t.kd = 1;
  EXPECT_NO_THROW(target_frame(1, t, {}));
  t.kd = std::nextafter(1.0, 2.0);
  EXPECT_THROW(target_frame(1, t, {}), std::invalid_argument);
}
TEST(RuntimeProtocol, FloatPositionModesSupportMultiTurnTargetsAndKeepExplicitLimits)
{
  for (Mode mode : {Mode::PositionVelocity, Mode::PositionForce}) {
    Target target;
    target.mode = mode;
    target.velocity_limit = 0.2;
    target.current_ratio = mode == Mode::PositionForce ? 0.1 : 0;
    for (double position : {-1885.0, 1885.0}) {
      target.position = position;
      const auto frame = target_frame(1, target, {});
      uint32_t raw = 0;
      for (unsigned j = 0; j < 4; ++j) {raw |= uint32_t(frame.data[j]) << (8 * j);}
      EXPECT_EQ(raw, float_bits(static_cast<float>(position)));
      EXPECT_THROW(target_frame(1, target, {}, {1, 0, 0, 500, 1}), std::invalid_argument);
    }
    target.position = std::nextafter(
      double(std::numeric_limits<float>::max()), std::numeric_limits<double>::infinity());
    EXPECT_THROW(target_frame(1, target, {}), std::invalid_argument);
  }
  Target mit;
  mit.position = 25;
  EXPECT_THROW(target_frame(1, mit, {}), std::invalid_argument);
}
TEST(RuntimeProtocol, LegacySerialCompatibilityPreservesAllFourModes)
{
  for (Mode mode : {Mode::Mit, Mode::PositionVelocity, Mode::Velocity, Mode::PositionForce}) {
    Target target; target.mode = mode;
    if (mode == Mode::Velocity) {target.velocity = -1;}
    else if (mode != Mode::Mit) {target.position = 1; target.velocity_limit = 0.5;}
    const auto native = target_frame(2, target, {});
    auto dirty = native;
    if (mode == Mode::Velocity) {
      std::fill(dirty.data.begin() + 4, dirty.data.end(), 0xab);
    }
    const auto padded = legacy_serial_frame(dirty);
    EXPECT_EQ(padded.id, native.id); EXPECT_EQ(padded.length, 8);
    const auto packet = serial_packet(padded);
    EXPECT_EQ(packet[13], native.id & 255); EXPECT_EQ(packet[14], native.id >> 8);
    EXPECT_EQ(packet[18], 8);
    EXPECT_TRUE(std::equal(native.data.begin(), native.data.begin() + native.length,
      packet.begin() + 21));
    if (mode == Mode::Velocity) {
      EXPECT_EQ(native.length, 4);  // Native CAN backends keep the true payload length.
      EXPECT_EQ(padded.data, (std::array<uint8_t, 8>{0, 0, 0x80, 0xbf, 0, 0, 0, 0}));
      EXPECT_TRUE(std::all_of(packet.begin() + 25, packet.begin() + 29,
        [](uint8_t b) {return b == 0;}));
    }
  }
  Frame short_read{0x7ff, {2, 0, 0x33, 60, 1, 2, 3, 4}};
  short_read.length = 4;
  EXPECT_EQ(legacy_serial_frame(short_read).data,
    (std::array<uint8_t, 8>{2, 0, 0x33, 60, 0, 0, 0, 0}));
  short_read.fd = true;
  EXPECT_THROW(legacy_serial_frame(short_read), std::invalid_argument);
  short_read.fd = false; short_read.length = 5;
  EXPECT_THROW(legacy_serial_frame(short_read), std::invalid_argument);
}
TEST_F(RuntimeTest, AsynchronousStartupConfirmsMappingsWatchdogAndDisabled)
{
  EXPECT_FALSE(bus->ready());
  EXPECT_THROW(bus->enable("a", now), std::runtime_error);
  ready();
  for (const auto & c : bus->configs()) {
    auto s = bus->snapshot(c.name, now);
    EXPECT_EQ(s.phase, Phase::Disabled); EXPECT_EQ(s.feedback.status, 0);
    EXPECT_GT(s.feedback_count, 0u);
    EXPECT_EQ(s.mapping.p, c.mapping.p);
    EXPECT_EQ(transport->sim.register_value(c.id, 9), 4000u);
  }
  EXPECT_TRUE(std::none_of(transport->sent.begin(), transport->sent.end(),
    [](const Frame & f) {return f.id != 0x7ff && f.data[7] == 0xfc;}));
}
TEST_F(RuntimeTest, FeedbackObserverPreservesEveryFrameInOneReceiveBatch)
{
  ready();
  const auto before = bus->snapshot("a", now).feedback_count;
  std::vector<uint64_t> counts;
  bus->set_feedback_callback([&](const std::string & name, const MotorSnapshot & sample, double received) {
    if (name == "a") {
      counts.push_back(sample.feedback_count);
      EXPECT_LE(received, now);
    }
  });
  for (unsigned i = 0; i < 10; ++i) {transport->sim.send(query_frame(1));}
  step();
  ASSERT_GE(counts.size(), 10u);
  EXPECT_EQ(counts.size(), bus->snapshot("a", now).feedback_count - before);
  for (size_t i = 0; i < counts.size(); ++i) {EXPECT_EQ(counts[i], before + i + 1);}
}
TEST_F(RuntimeTest, MultiMotorLatestTargetsAndIndependentStop)
{
  ready(); enable("a"); enable("b");
  Target a; a.torque = 0.5; Target b; b.torque = -0.4;
  bus->command("a", a, now); bus->command("b", b, now);
  a.torque = 0.2; bus->command("a", a, now);
  transport->sent.clear(); step(2);
  ASSERT_GE(transport->sent.size(), 4u);
  EXPECT_EQ(transport->sent[0].id, 1); EXPECT_EQ(transport->sent[1].id, 2);
  EXPECT_NEAR(decode_mit(transport->sent[0], {}).t, 0.2, 0.006);
  bus->disable("a", now); step();
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
  EXPECT_EQ(bus->snapshot("b", now).phase, Phase::Enabled);
}
TEST_F(RuntimeTest, ExplicitStartupModeConfigurationEnablesMultiTurnTargetsWithoutAutoEnable)
{
  auto desired = configuration();
  for (auto & config : desired) {
    config.mode = Mode::PositionVelocity;
    config.configure_mode_on_startup = true;
  }
  auto remote = std::make_unique<RecordingTransport>(configuration());
  transport = remote.get();
  bus = std::make_unique<Bus>(std::move(remote), desired, true);
  for (unsigned i = 0; i < 500 && !bus->ready(); ++i) {step();}
  ASSERT_TRUE(bus->ready()) << bus->initialization_error();
  for (const auto & config : desired) {
    EXPECT_EQ(transport->sim.register_value(config.id, 10), 2u);
    EXPECT_EQ(bus->snapshot(config.name, now).phase, Phase::Disabled);
    EXPECT_EQ(bus->snapshot(config.name, now).feedback.status, 0);
  }
  EXPECT_TRUE(std::none_of(transport->sent.begin(), transport->sent.end(), [](const Frame & frame) {
    return (frame.id != 0x7ff && frame.data[7] == 0xfc) ||
      (frame.id == 0x7ff && frame.data[2] == 0xaa);
  }));
  enable("a");
  for (const double position : {8 * std::acos(-1.0), -8 * std::acos(-1.0)}) {
    Target target; target.mode = Mode::PositionVelocity;
    target.position = position; target.velocity_limit = 10;
    for (unsigned i = 0; i < 1200; ++i) {
      bus->command("a", target, now); step();
    }
    EXPECT_NEAR(bus->snapshot("a", now).absolute_position, position, 0.02);
    EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Enabled);
  }
}

TEST_F(RuntimeTest, StartupModeWriteRequiresIndependentReadback)
{
  auto desired = configuration();
  desired[0].mode = Mode::PositionVelocity;
  desired[0].configure_mode_on_startup = true;
  auto remote = std::make_unique<RecordingTransport>(configuration());
  transport = remote.get(); transport->ignore_write = true;
  bus = std::make_unique<Bus>(std::move(remote), desired, true);
  step(500);
  EXPECT_FALSE(bus->ready());
  EXPECT_FALSE(bus->initialization_error().empty());
  EXPECT_EQ(transport->sim.register_value(1, 10), 1u);
  EXPECT_TRUE(std::none_of(transport->sent.begin(), transport->sent.end(), [](const Frame & frame) {
    return frame.id != 0x7ff && frame.data[7] == 0xfc;
  }));
}

TEST_F(RuntimeTest, StartupModeConfigurationNeverWritesWhileMotorIsEnabled)
{
  auto desired = configuration();
  desired[0].mode = Mode::PositionVelocity;
  desired[0].configure_mode_on_startup = true;
  auto remote = std::make_unique<RecordingTransport>(configuration());
  transport = remote.get();
  transport->sim.send(special_frame(1, 0xfc));
  bus = std::make_unique<Bus>(std::move(remote), desired, true);
  step(500);
  EXPECT_FALSE(bus->ready());
  EXPECT_TRUE(std::none_of(transport->sent.begin(), transport->sent.end(), [](const Frame & frame) {
    return frame.id == 0x7ff && frame.data[2] == 0x55 && frame.data[3] == 10;
  }));
}
TEST_F(RuntimeTest, StartupAnchorsEachMotorOnceAtMultiTurnPosition)
{
  transport->sim.set_position(1, 1884.98);
  transport->sim.set_position(2, -50.0);
  ready();
  for (const auto & config : bus->configs()) {
    const auto state = bus->snapshot(config.name, now);
    EXPECT_TRUE(state.absolute_position_valid);
    EXPECT_NEAR(state.absolute_position, config.id == 1 ? 1884.98 : -50.0, 0.001);
    EXPECT_EQ(state.position_wraps, 0);
    EXPECT_LE(std::abs(state.feedback.p), state.mapping.p);
  }
  step(50);
  for (uint8_t id : {1, 2}) {
    EXPECT_EQ(std::count_if(transport->sent.begin(), transport->sent.end(),
      [id](const Frame & f) {
        return f.id == 0x7ff && f.data[0] == id && f.data[2] == 0x33 && f.data[3] == 80;
      }), 1);
  }
}
TEST_F(RuntimeTest, StartupPositionAnchorWaitsForBusMaintenanceToFinish)
{
  transport->motion_available = false;
  step(150);
  EXPECT_FALSE(bus->ready());
  EXPECT_TRUE(bus->initialization_error().empty());
  EXPECT_FALSE(bus->snapshot("a", now).absolute_position_valid);
  const auto position_reads = [&]() {
      return std::count_if(transport->sent.begin(), transport->sent.end(), [](const Frame & f) {
        return f.id == 0x7ff && f.data[2] == 0x33 && f.data[3] == 80;
      });
    };
  EXPECT_EQ(position_reads(), 0);
  transport->motion_available = true;
  step(10);
  EXPECT_EQ(position_reads(), 0);
  transport->motion_available = false;
  step(10);
  transport->motion_available = true;
  step(10);
  EXPECT_EQ(position_reads(), 0);
  ready();
  EXPECT_TRUE(bus->snapshot("a", now).absolute_position_valid);
  EXPECT_TRUE(bus->snapshot("b", now).absolute_position_valid);
  EXPECT_EQ(position_reads(), 2);
}
TEST_F(RuntimeTest, SoftwarePositionTracksThreeHundredTurnsAndReverseWithoutApiCorrection)
{
  transport->sim.set_position(1, 0.02);
  ready();
  const double start = bus->snapshot("a", now).absolute_position;
  for (unsigned direction : {0, 1}) {
    for (unsigned j = 1; j <= 3000; ++j) {
      const double turns = direction == 0 ? j / 10.0 : 300 - j / 10.0;
      transport->sim.set_position(1, 0.02 + turns * 2 * std::acos(-1.0));
      step(2);
      const auto state = bus->snapshot("a", now);
      ASSERT_TRUE(state.absolute_position_valid) << state.error;
      ASSERT_NEAR(state.absolute_position, 0.02 + turns * 2 * std::acos(-1.0), 0.001);
    }
    EXPECT_EQ(bus->snapshot("a", now).position_wraps, direction == 0 ? 75 : 0);
  }
  EXPECT_NEAR(bus->snapshot("a", now).absolute_position, start, 0.001);
  EXPECT_EQ(std::count_if(transport->sent.begin(), transport->sent.end(), [](const Frame & f) {
    return f.id == 0x7ff && f.data[0] == 1 && f.data[2] == 0x33 && f.data[3] == 80;
  }), 1);
}
TEST_F(RuntimeTest, MissingOrNonfinitePositionAnchorPreventsReadiness)
{
  transport->drop_position_read = true;
  step(300);
  EXPECT_FALSE(bus->ready());
  EXPECT_FALSE(bus->snapshot("a", now).absolute_position_valid);
  EXPECT_NE(bus->initialization_error().find("register=80"), std::string::npos);
}
TEST_F(RuntimeTest, NonfinitePositionAnchorIsExplicitStartupFailure)
{
  transport->position_raw_override = float_bits(std::numeric_limits<float>::quiet_NaN());
  step(200);
  EXPECT_FALSE(bus->ready());
  EXPECT_NE(bus->initialization_error().find("p_m position is not finite"), std::string::npos);
}
TEST_F(RuntimeTest, InconsistentPositionAnchorIsExplicitStartupFailure)
{
  transport->position_raw_override = float_bits(5);
  step(200);
  EXPECT_FALSE(bus->ready());
  EXPECT_NE(bus->initialization_error().find("differs from compact"), std::string::npos);
}
TEST_F(RuntimeTest, LongFeedbackGapInvalidatesAbsolutePositionAndCannotResetIt)
{
  ready();
  transport->sim.receive();
  transport->sim.drop_replies(true);
  step(100);
  EXPECT_FALSE(bus->snapshot("a", now).absolute_position_valid);
  transport->sim.drop_replies(false);
  step(3);
  const auto state = bus->snapshot("a", now);
  EXPECT_EQ(state.phase, Phase::Fault);
  EXPECT_FALSE(state.absolute_position_valid);
  EXPECT_NE(state.error.find("absolute position ambiguous"), std::string::npos);
  EXPECT_THROW(bus->reset("a", now), std::runtime_error);
  EXPECT_THROW(bus->enable("a", now), std::runtime_error);
}
TEST_F(RuntimeTest, ConfirmedDeviceZeroReanchorsSoftwareWithoutReadingRegisterAgain)
{
  transport->sim.set_position(1, 50);
  ready();
  EXPECT_NEAR(bus->snapshot("a", now).absolute_position, 50, 0.001);
  EXPECT_EQ(await(bus->set_device_zero("a", now)).state, OperationState::Confirmed);
  step(3);
  const auto state = bus->snapshot("a", now);
  EXPECT_TRUE(state.absolute_position_valid);
  EXPECT_NEAR(state.absolute_position, 0, 0.001);
  EXPECT_EQ(state.position_wraps, 0);
}
TEST_F(RuntimeTest, PositionModeEnableUsesFullContinuousNeutralPosition)
{
  auto config = configuration();
  config[0].mode = Mode::PositionVelocity;
  auto sim = std::make_unique<RecordingTransport>(config);
  auto * raw = sim.get();
  raw->sim.set_position(1, 50);
  Bus local(std::move(sim), config, true);
  double time = 0;
  for (unsigned j = 0; j < 200; ++j) {local.tick(time += 0.005);}
  ASSERT_TRUE(local.ready()) << local.initialization_error();
  raw->sent.clear();
  EXPECT_NO_THROW(local.enable("a", time));
  ASSERT_EQ(raw->sent.size(), 2u);
  EXPECT_EQ(raw->sent[0].id, 0x101);
  EXPECT_TRUE(std::equal(raw->sent[0].data.begin(), raw->sent[0].data.begin() + 4,
    (std::array<uint8_t, 4>{0, 0, 0x48, 0x42}).begin()));
  local.tick(time += 0.005);
  EXPECT_EQ(local.snapshot("a", time).phase, Phase::Enabled);
  Target target;
  target.mode = Mode::PositionVelocity; target.position = -1885; target.velocity_limit = 0.2;
  EXPECT_NO_THROW(local.command("a", target, time));
  local.tick(time += 0.005);
  EXPECT_EQ(local.snapshot("a", time).phase, Phase::Enabled);
  EXPECT_NEAR(local.snapshot("a", time).absolute_position, 50, 0.001);
}
TEST_F(RuntimeTest, IndependentReadbackNotWriteEchoConfirms)
{
  ready();
  const auto id = bus->write_register("a", 25, float_bits(3), now);
  EXPECT_EQ(bus->operation(id).state, OperationState::Pending);
  step(2);
  EXPECT_EQ(bus->operation(id).state, OperationState::Pending);
  auto result = await(id);
  EXPECT_EQ(result.state, OperationState::Confirmed);
  EXPECT_EQ(result.raw_value, float_bits(3));
  const auto write = std::find_if(transport->sent.begin(), transport->sent.end(),
    [](const Frame & f) {return f.data[2] == 0x55 && f.data[3] == 25;});
  ASSERT_NE(write, transport->sent.end());
  EXPECT_TRUE(std::any_of(write + 1, transport->sent.end(),
    [](const Frame & f) {return f.data[2] == 0x33 && f.data[3] == 25;}));
  auto read = await(bus->read_register("a", 25, now));
  EXPECT_EQ(read.state, OperationState::Confirmed); EXPECT_EQ(read.raw_value, float_bits(3));
}
TEST_F(RuntimeTest, EchoWithoutAppliedWriteIsRejected)
{
  ready(); transport->ignore_write = true;
  const auto id = bus->write_register("a", 25, float_bits(3), now);
  EXPECT_EQ(await(id).state, OperationState::Rejected);
  EXPECT_TRUE(bus->ready());
}
TEST_F(RuntimeTest, ParameterResponsesNeverBecomeMotorMeasurements)
{
  ready();
  auto before = bus->snapshot("a", now).feedback_count;
  transport->sim.inject_frame({0, {1, 0, 0x33, 25, 0xff, 0xff, 0xff, 0xff}});
  transport->sim.inject_frame({0, {1, 0, 0x55, 25, 0xff, 0xff, 0xff, 0xff}});
  transport->sim.inject_frame({0, {1, 0, 0xaa, 1, 0, 0, 0, 0}});
  // Drop ordinary query responses, leaving only synthetic parameter packets.
  transport->sim.receive();
  transport->sim.drop_replies(true);
  transport->sim.inject_frame({0, {1, 0, 0x33, 25, 0xff, 0xff, 0xff, 0xff}});
  step();
  EXPECT_EQ(bus->snapshot("a", now).feedback_count, before);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
}
TEST_F(RuntimeTest, OnlineMaintenanceRejectedBusWide)
{
  ready(); enable("a");
  EXPECT_EQ(bus->operation(bus->read_register("b", 60, now)).state, OperationState::Rejected);
  EXPECT_EQ(bus->operation(bus->write_register("b", 25, float_bits(1), now)).state, OperationState::Rejected);
  EXPECT_EQ(bus->operation(bus->save_parameters("b", now)).state, OperationState::Rejected);
}
TEST_F(RuntimeTest, ProtectsReservedReadOnlyRiskAndOutOfRangeWrites)
{
  ready();
  for (uint8_t rid : {7, 8, 9, 10, 21, 22, 23, 35, 37, 0, 29, 30, 200}) {
    EXPECT_EQ(bus->operation(bus->write_register("a", rid, 1, now)).state,
      OperationState::Rejected) << unsigned(rid);
  }
  EXPECT_EQ(bus->operation(bus->write_register("a", 24, float_bits(99), now)).state, OperationState::Rejected);
  EXPECT_EQ(bus->operation(bus->write_register("a", 32, float_bits(500), now)).state, OperationState::Rejected);
  EXPECT_EQ(bus->operation(bus->write_register("a", 25, float_bits(-1), now)).state, OperationState::Rejected);
  EXPECT_EQ(bus->operation(bus->write_register("a", 25, 0x7fc00000, now)).state, OperationState::Rejected);
}
TEST_F(RuntimeTest, SaveWaitsForAcknowledgementAndFlashBudget)
{
  ready();
  auto id = bus->save_parameters("a", now);
  step(3);
  EXPECT_EQ(bus->operation(id).state, OperationState::Pending);
  EXPECT_EQ(await(id).state, OperationState::Confirmed);
  const auto found = std::find_if(transport->sent.begin(), transport->sent.end(),
    [](const Frame & f) {return f.id == 0x7ff && f.data[2] == 0xaa;});
  ASSERT_NE(found, transport->sent.end()); EXPECT_EQ(found->data[3], 1);
}
TEST_F(RuntimeTest, ModeChangesAllFourRequireExplicitEnableAndNewTargets)
{
  ready();
  for (Mode mode : {Mode::PositionVelocity, Mode::Velocity, Mode::PositionForce, Mode::Mit}) {
    const auto id = bus->set_mode("a", mode, now);
    EXPECT_EQ(await(id).state, OperationState::Confirmed);
    auto s = bus->snapshot("a", now);
    EXPECT_EQ(s.mode, mode); EXPECT_EQ(s.phase, Phase::Disabled);
    EXPECT_EQ(bus->configs()[0].mode, mode);
    // CONFIRMED is a terminal maintenance result, not a hidden cooldown.
    enable("a");
    Target t; t.mode = mode;
    if (mode == Mode::Velocity) {t.velocity = 0.1;}
    else if (mode != Mode::Mit) {t.position = 0.01; t.velocity_limit = 0.2; t.current_ratio =
      mode == Mode::PositionForce ? 0.1 : 0;}
    bus->command("a", t, now); step(2);
    EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Enabled);
    EXPECT_EQ(bus->operation(bus->set_mode("a", Mode::Mit, now)).state, OperationState::Rejected);
    bus->disable("a", now); step(4);
  }
}
TEST_F(RuntimeTest, DeviceFaultClearReturnsDisabledWithoutRedundantHostReset)
{
  ready(); transport->sim.inject_fault(1, 0xb); step();
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_TRUE(bus->snapshot("a", now).device_fault);
  EXPECT_FALSE(bus->snapshot("a", now).host_fault);
  const auto id = bus->clear_motor_fault("a", now);
  EXPECT_EQ(await(id).state, OperationState::Confirmed);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
  EXPECT_EQ(bus->snapshot("a", now).feedback.status, 0);
  EXPECT_FALSE(bus->snapshot("a", now).device_fault);
  EXPECT_FALSE(bus->snapshot("a", now).host_fault);
  EXPECT_TRUE(std::any_of(transport->sent.begin(), transport->sent.end(),
    [](const Frame & f) {return f.id == 1 && f.data[7] == 0xfb;}));
}
TEST_F(RuntimeTest, ClearDeviceFaultDoesNotEraseIndependentHostCommunicationFault)
{
  ready();
  transport->sim.inject_fault(1, 0xa); step();
  bus->fail("a", "independent host stop");
  const auto operation = bus->clear_motor_fault("a", now);
  EXPECT_EQ(await(operation).state, OperationState::Confirmed);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_TRUE(bus->snapshot("a", now).host_fault);
  EXPECT_FALSE(bus->snapshot("a", now).device_fault);
  EXPECT_EQ(bus->snapshot("a", now).error, "independent host stop");
  step(3); bus->reset("a", now);
  EXPECT_FALSE(bus->snapshot("a", now).host_fault);
}

TEST_F(RuntimeTest, FirmwareProtectionDoesNotCauseRepeatedHostDisableTraffic)
{
  ready(); enable("a");
  transport->sent.clear();
  transport->sim.inject_fault(1, 0xb);
  step(20);
  const auto stops = std::count_if(transport->sent.begin(), transport->sent.end(), [](const Frame & frame) {
    return frame.id == 1 && frame.data[7] == 0xfd;
  });
  EXPECT_EQ(stops, 1);
  EXPECT_TRUE(bus->snapshot("a", now).device_fault);
  EXPECT_FALSE(bus->snapshot("a", now).host_fault);
}
TEST_F(RuntimeTest, DeviceZeroRequiresFreshRelevantFeedback)
{
  ready(); const auto id = bus->set_device_zero("a", now);
  step(2);
  EXPECT_EQ(bus->operation(id).state, OperationState::Pending);
  EXPECT_EQ(await(id).state, OperationState::Confirmed);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
  EXPECT_TRUE(std::any_of(transport->sent.begin(), transport->sent.end(),
    [](const Frame & f) {return f.id == 1 && f.data[7] == 0xfe;}));
}
TEST_F(RuntimeTest, TimeoutIsUnknownAndCannotBeSatisfiedByOldCache)
{
  ready(); EXPECT_EQ(await(bus->read_register("a", 60, now)).state, OperationState::Confirmed);
  step(4);
  transport->sim.drop_replies(true);
  const auto id = bus->read_register("a", 60, now);
  EXPECT_EQ(await(id).state, OperationState::Unknown);
  EXPECT_FALSE(bus->ready());
  EXPECT_EQ(bus->snapshot("b", now).phase, Phase::Fault);
}
TEST_F(RuntimeTest, FeedbackAndCommandTimeoutsLatchAndStop)
{
  ready(); enable("a"); enable("b");
  transport->sim.drop_replies(true); step(25);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_EQ(bus->snapshot("b", now).phase, Phase::Fault);
  EXPECT_NE(bus->snapshot("a", now).error.find("feedback timeout"), std::string::npos);
}
TEST_F(RuntimeTest, PublisherTimeoutWhileFeedbackFresh)
{
  ready(); enable("a"); step(55);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_NE(bus->snapshot("a", now).error.find("publisher"), std::string::npos);
  step(3); bus->reset("a", now); enable("a");
  const auto f = transport->sent[transport->sent.size() - 2];
  EXPECT_NEAR(decode_mit(f, {}).t, 0, 0.006);
}
TEST_F(RuntimeTest, TransportErrorsFaultEveryMotor)
{
  ready(); enable("a"); enable("b");
  transport->broken = true; step();
  EXPECT_FALSE(bus->ready());
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_EQ(bus->snapshot("b", now).phase, Phase::Fault);
  EXPECT_THROW(bus->enable("a", now), std::runtime_error);
  transport->broken = false;
  const auto before = bus->snapshot("a", now).feedback_count;
  step(15);
  EXPECT_GT(bus->snapshot("a", now).feedback_count, before);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_FALSE(bus->ready());
}
TEST_F(RuntimeTest, RejectsNonMonotonicAndNonfiniteCommandClock)
{
  ready(); enable("a");
  EXPECT_THROW(bus->command("a", {}, std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
  EXPECT_THROW(bus->command("a", {}, now - 0.1), std::runtime_error);
  EXPECT_THROW(bus->command("a", {}, now + 10), std::runtime_error);
  bus->tick(now - 1);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
}
TEST_F(RuntimeTest, StopTrafficPrecedesAnotherMotorsControl)
{
  ready(); enable("a"); enable("b"); transport->sent.clear();
  bus->fail("b", "test stop priority"); step();
  ASSERT_GE(transport->sent.size(), 2u);
  EXPECT_EQ(transport->sent[0].id, 2); EXPECT_EQ(transport->sent[0].data[7], 0xfd);
  EXPECT_EQ(transport->sent[1].id, 1);
}
TEST_F(RuntimeTest, SaveActuallyPersistsSimulationRegisters)
{
  ready();
  EXPECT_EQ(await(bus->write_register("a", 25, float_bits(4), now)).state, OperationState::Confirmed);
  EXPECT_EQ(await(bus->save_parameters("a", now)).state, OperationState::Confirmed);
  EXPECT_EQ(await(bus->write_register("a", 25, float_bits(5), now)).state, OperationState::Confirmed);
  transport->sim.power_cycle(1);
  EXPECT_EQ(transport->sim.register_value(1, 25), float_bits(4));
}
TEST_F(RuntimeTest, CommandLimitsDoNotSubstituteForFeedbackProtection)
{
  ready();
  transport->sim.inject_frame(feedback_frame(1, 0, {0, 0, 2, 0, 28, 29}, {}));
  // Remove previously queued normal query feedback so the injected frame is newest.
  auto queued = transport->sim.receive();
  for (const auto & frame : queued) {transport->injected.push_back(frame);}
  step();
  EXPECT_EQ(bus->operation(bus->set_mode("a", Mode::Velocity, now)).state, OperationState::Rejected);
  auto configs = configuration();
  configs[0].limits = {1, 0.2, 0.1, 5, 1};
  auto sim = std::make_unique<SimBusTransport>(configs); auto * raw = sim.get();
  Bus limited(std::move(sim), configs, true);
  double time = 0;
  for (unsigned i = 0; i < 150; ++i) {limited.tick(time += 0.005);}
  ASSERT_TRUE(limited.ready()); limited.enable("a", time); limited.tick(time += 0.005);
  EXPECT_EQ(limited.snapshot("a", time).phase, Phase::Enabled);
  raw->inject_frame(feedback_frame(1, 0, {1, 2, 0.3, 0.15, 28, 29}, {}));
  limited.tick(time += 0.005);
  EXPECT_EQ(limited.snapshot("a", time).phase, Phase::Enabled);
  EXPECT_TRUE(limited.snapshot("a", time).error.empty());
  Target invalid;
  invalid.position = 1.1;
  EXPECT_THROW(limited.command("a", invalid, time), std::invalid_argument);
  invalid = {}; invalid.velocity = 0.3;
  EXPECT_THROW(limited.command("a", invalid, time), std::invalid_argument);
  invalid = {}; invalid.torque = 0.15;
  EXPECT_THROW(limited.command("a", invalid, time), std::invalid_argument);
  Target impedance;
  impedance.position = 0.03; impedance.kp = 5; impedance.kd = 1;
  EXPECT_NO_THROW(limited.command("a", impedance, time));
}
TEST_F(RuntimeTest, PendingAndHistoryBoundedAndUnknownIdsExplicit)
{
  ready();
  std::vector<uint64_t> ids;
  for (unsigned i = 0; i < 400; ++i) {ids.push_back(bus->read_register("a", 60, now));}
  unsigned pending = 0;
  for (const auto id : ids) {pending += bus->operation(id).state == OperationState::Pending;}
  EXPECT_LE(pending, 64u);
  EXPECT_EQ(bus->operation(999999).state, OperationState::Unknown);
  EXPECT_EQ(bus->operation(ids.back()).state, OperationState::Rejected);
}

TEST_F(RuntimeTest, RemoteTransportFailureTerminatesPendingOperationsAsUnknown)
{
  ready();
  const auto operation = bus->read_register("a", 60, now);
  ASSERT_EQ(bus->operation(operation).state, OperationState::Pending);
  bus->transport_failed("remote CAN session lost");
  EXPECT_EQ(bus->operation(operation).state, OperationState::Unknown);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Fault);
  EXPECT_EQ(bus->operation(operation).message, "remote CAN session lost");
  EXPECT_FALSE(bus->ready());
}

TEST_F(RuntimeTest, MaintenanceReservationLossRejectsUnsentModeWithoutLatchingFault)
{
  ready();
  const auto operation = bus->set_mode("a", Mode::Velocity, now);
  ASSERT_EQ(bus->operation(operation).state, OperationState::Pending);
  transport->available = false; transport->reserved = false;
  step();
  EXPECT_EQ(bus->operation(operation).state, OperationState::Rejected);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
  EXPECT_EQ(bus->snapshot("a", now).mode, Mode::Mit);
  EXPECT_TRUE(bus->ready());
}

TEST_F(RuntimeTest, StartupDeviceDeadlineBeginsOnlyAfterMaintenanceReservation)
{
  transport->reserved = false;
  step(240);
  EXPECT_FALSE(bus->ready());
  EXPECT_TRUE(bus->initialization_error().empty());
  EXPECT_TRUE(transport->sent.empty());
  transport->reserved = true;
  ready();
}

TEST_F(RuntimeTest, QuietReservationWaitIsBoundedAndDoesNotWriteDevice)
{
  ready();
  transport->sent.clear();
  transport->reserved = false;
  const auto operation = bus->write_register("a", 25, float_bits(2), now);
  step(420);
  EXPECT_EQ(bus->operation(operation).state, OperationState::Rejected);
  EXPECT_NE(bus->operation(operation).message.find("before any device request"), std::string::npos);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
  EXPECT_TRUE(bus->ready());
  EXPECT_FALSE(std::any_of(transport->sent.begin(), transport->sent.end(), [](const Frame & frame) {
    return frame.id == 0x7ff && frame.data[2] == 0x55;
  }));
}

TEST_F(RuntimeTest, StopOnFreshDisabledMotorIsIdempotentAndStillTransmitsStop)
{
  ready();
  transport->sent.clear();
  bus->disable("a", now);
  EXPECT_EQ(bus->snapshot("a", now).phase, Phase::Disabled);
  ASSERT_FALSE(transport->sent.empty());
  EXPECT_EQ(transport->sent.back().data[7], 0xfd);
}
TEST(RuntimeInitialization, RejectsUnsafeConfigurationAndModeMismatch)
{
  auto c = configuration(); c[1].id = 1;
  EXPECT_THROW(Bus(make_sim_transport(configuration()), c, true), std::invalid_argument);
  c = configuration(); c[1].name = "a";
  EXPECT_THROW(Bus(make_sim_transport(configuration()), c, true), std::invalid_argument);
  c = configuration(); c[1].master = 0x101;
  EXPECT_THROW(Bus(make_sim_transport(configuration()), c, true), std::invalid_argument);
  c = configuration(); c[1].watchdog = 0;
  EXPECT_THROW(Bus(make_sim_transport(configuration()), c, true), std::invalid_argument);
  c = configuration(); c[0].mode = Mode::Velocity;
  Bus b(make_sim_transport(configuration()), c, false);
  for (unsigned i = 0; i < 150; ++i) {b.tick(i * 0.005);}
  EXPECT_FALSE(b.ready());
  EXPECT_NE(b.initialization_error().find("CTRL_MODE mismatch"), std::string::npos);
}
TEST(RuntimeInitialization, MissingStartupReplyFailsExplicitlyWithoutEnable)
{
  auto t = std::make_unique<RecordingTransport>(configuration()); auto * raw = t.get();
  raw->sim.drop_replies(true);
  Bus b(std::move(t), configuration(), false);
  for (unsigned i = 0; i < 150; ++i) {b.tick(i * 0.005);}
  EXPECT_FALSE(b.ready()); EXPECT_FALSE(b.initialization_error().empty());
  EXPECT_TRUE(std::none_of(raw->sent.begin(), raw->sent.end(),
    [](const Frame & f) {return f.id != 0x7ff && f.data[7] == 0xfc;}));
}
TEST(RuntimeInitialization, InvalidSocketInterfaceRejectedWithoutHardwareAccess)
{
  EXPECT_THROW(make_socketcan_transport(""), std::invalid_argument);
  EXPECT_THROW(make_socketcan_transport(std::string(100, 'x')), std::invalid_argument);
}
