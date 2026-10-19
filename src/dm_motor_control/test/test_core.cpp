#include "dm_motor_control/core.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <gtest/gtest.h>

using namespace dm_motor;

namespace
{
class FakeTransport : public Transport
{
public:
  void send(const Frame & f) override {sent.push_back(f);}
  std::vector<Frame> receive() override
  {
    auto result = std::move(incoming);
    incoming.clear();
    return result;
  }
  void feedback(uint8_t status, uint8_t id = 7, uint16_t master = 0)
  {
    incoming.push_back(feedback_frame(id, master, {status, 0, 0, 0, 28, 29}, {}));
  }
  std::vector<Frame> sent, incoming;
};
class RegisterTransport : public FakeTransport
{
public:
  RegisterTransport()
  {
    registers = {{7, 42}, {9, 0}, {10, 1}, {21, bits(12.5f)}, {22, bits(30.f)}, {23, bits(10.f)}};
  }
  static uint32_t bits(float value)
  {
    uint32_t result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
  }
  void send(const Frame & f) override
  {
    FakeTransport::send(f);
    if (f.id != 0x7ff) {return;}
    if (f.data[2] == 0xcc) {feedback(status, 7, 42); return;}
    const auto rid = f.data[3];
    if (f.data[2] == 0x55 && apply_write) {
      registers[rid] = 0;
      for (unsigned i = 0; i < 4; ++i) {
        registers[rid] |= uint32_t(f.data[4 + i]) << (8 * i);
      }
    }
    Frame reply = f;
    reply.id = 42;
    const auto value = f.data[2] == 0x55 && !apply_write ? 4000u : registers.at(rid);
    for (unsigned i = 0; i < 4; ++i) {reply.data[4 + i] = (value >> (8 * i)) & 255;}
    incoming.push_back(reply);
  }
  std::map<uint8_t, uint32_t> registers;
  uint8_t status{0};
  bool apply_write{true};
};
std::array<uint8_t, 16> rx_packet(const Frame & f, uint8_t command = 0x11)
{
  std::array<uint8_t, 16> result{0xaa, command, 8};
  result[3] = f.id & 255;
  result[4] = f.id >> 8;
  std::copy(f.data.begin(), f.data.end(), result.begin() + 7);
  result[15] = 0x55;
  return result;
}
void enabled(Controller & c, FakeTransport & t)
{
  t.feedback(0);
  c.tick(0.0);
  c.enable(0.01);
  t.feedback(1);
  c.tick(0.02);
  ASSERT_EQ(c.phase(), Phase::Enabled);
}
}

TEST(Protocol, GoldenMitBytesAndRoundTrip)
{
  auto f = mit_frame(7, {}, {});
  const std::array<uint8_t, 8> expected{0x7f, 0xff, 0x7f, 0xf0, 0, 0, 0x07, 0xff};
  EXPECT_EQ(f.id, 7);
  EXPECT_EQ(f.data, expected);
  const Command input{0.4, -0.7, 12.0, 1.0, 0.3};
  auto output = decode_mit(mit_frame(7, input, {}), {});
  EXPECT_NEAR(input.p, output.p, 25.0 / 65535);
  EXPECT_NEAR(input.v, output.v, 60.0 / 4095);
  EXPECT_NEAR(input.kp, output.kp, 500.0 / 4095);
  EXPECT_NEAR(input.kd, output.kd, 5.0 / 4095);
  EXPECT_NEAR(input.t, output.t, 20.0 / 4095);
}
TEST(Protocol, FeedbackRoutingAndFaultCode)
{
  auto f = feedback_frame(7, 42, {0xb, 0.2, -0.5, 0.1, 60, 70}, {});
  auto fb = decode_feedback(f, 7, 42, {});
  ASSERT_TRUE(fb);
  EXPECT_EQ(fb->status, 0xb);
  EXPECT_EQ(fb->rotor, 70);
  EXPECT_NEAR(fb->p, 0.2, 25.0 / 65535);
  EXPECT_FALSE(decode_feedback(f, 8, 42, {}));
  EXPECT_FALSE(decode_feedback(f, 7, 43, {}));
}
TEST(Protocol, RejectsInvalidValuesButAllowsZeroDamping)
{
  Command c;
  c.p = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_command(c, {}, {}), std::invalid_argument);
  c = {}; c.t = std::numeric_limits<double>::infinity();
  EXPECT_THROW(validate_command(c, {}, {}), std::invalid_argument);
  c = {}; c.v = 30.01;
  EXPECT_THROW(validate_command(c, {}, {}), std::invalid_argument);
  c = {}; c.kp = 1;
  EXPECT_NO_THROW(validate_command(c, {}, {}));
  c.kd = 0.0001;
  EXPECT_NO_THROW(validate_command(c, {}, {}));
  c.kd = 1;
  EXPECT_NO_THROW(validate_command(c, {}, {}));
}
TEST(Protocol, DefaultRangesFollowActualMappingAndAllowFullEndpoints)
{
  const Mapping mapping{12.5, 50, 10};
  const auto limits = resolve_limits(mapping, {});
  EXPECT_EQ(limits.p, 12.5);
  EXPECT_EQ(limits.v, 50);
  EXPECT_EQ(limits.t, 10);
  EXPECT_EQ(limits.kp, 500);
  EXPECT_EQ(limits.kd, 1);
  const Command commands[] = {{12.5, 50, 500, 1, -10}, {-12.5, -50, 0, 0, 10},
    {12.5, 50, 500, 0, 10}, {2, 4, 30, 0, 2}, {0, 0, 500, 1, 0}};
  for (const auto & command : commands) {
    ASSERT_NO_THROW(validate_command(command, mapping, {}));
    const auto decoded = decode_mit(mit_frame(7, command, mapping), mapping);
    EXPECT_NEAR(decoded.p, command.p, 25.0 / 65535);
    EXPECT_NEAR(decoded.v, command.v, 100.0 / 4095);
    EXPECT_NEAR(decoded.kp, command.kp, 500.0 / 4095);
    EXPECT_NEAR(decoded.kd, command.kd, 5.0 / 4095);
    EXPECT_NEAR(decoded.t, command.t, 20.0 / 4095);
  }
}
TEST(Protocol, ExplicitLowerLimitsRemainAvailable)
{
  const Limits limits{1, 1, 0.5, 20, 0.5};
  EXPECT_NO_THROW(validate_command({1, 1, 20, 0.5, 0.5}, {}, limits));
  EXPECT_THROW(validate_command({1.01, 0, 0, 0, 0}, {}, limits), std::invalid_argument);
  EXPECT_THROW(validate_command({0, 1.01, 0, 0, 0}, {}, limits), std::invalid_argument);
  EXPECT_THROW(validate_command({0, 0, 0, 0, 0.51}, {}, limits), std::invalid_argument);
  EXPECT_THROW(validate_command({0, 0, 20.01, 0, 0}, {}, limits), std::invalid_argument);
  EXPECT_THROW(validate_command({0, 0, 0, 0.51, 0}, {}, limits), std::invalid_argument);
  EXPECT_THROW(resolve_limits({}, {-1, 0, 0, 500, 1}), std::invalid_argument);
  EXPECT_THROW(resolve_limits({}, {0, 31, 0, 500, 1}), std::invalid_argument);
  EXPECT_THROW(resolve_limits({}, {0, 0, 0, 501, 1}), std::invalid_argument);
}
TEST(Protocol, DampingCapPreservesWireScale)
{
  const Command command{0, 0, 0, 1, 0};
  const auto frame = mit_frame(7, command, {});
  EXPECT_EQ((frame.data[5] << 4) | (frame.data[6] >> 4), 819);
  EXPECT_DOUBLE_EQ(decode_mit(frame, {}).kd, 1);
  auto invalid = command;
  invalid.kd = std::nextafter(1.0, 2.0);
  EXPECT_THROW(mit_frame(7, invalid, {}), std::invalid_argument);
  EXPECT_THROW(validate_command(invalid, {}, {}), std::invalid_argument);
  EXPECT_THROW(resolve_limits({}, {0, 0, 0, 500, invalid.kd}), std::invalid_argument);
  Frame legacy;
  legacy.data.fill(0xff);
  EXPECT_DOUBLE_EQ(decode_mit(legacy, {}).kd, 5);
}
TEST(Protocol, UnsafeDampingAndSpecialCommandPrefixesAreRejected)
{
  const Mapping mapping{12.5, 50, 10};
  for (unsigned code = 4091; code <= 4095; ++code) {
    const double torque = code == 4095 ? mapping.t :
      std::nextafter(-mapping.t + code * (2 * mapping.t) / 4095,
        std::numeric_limits<double>::infinity());
    const Command command{mapping.p, mapping.v, 500, 5, torque};
    EXPECT_THROW(mit_frame(7, command, mapping), std::invalid_argument);
    EXPECT_THROW(validate_command(command, mapping, {}), std::invalid_argument);
  }
}
TEST(Serial, GoldenPacket)
{
  auto packet = serial_packet(special_frame(7, 0xfc));
  EXPECT_EQ(packet[0], 0x55);
  EXPECT_EQ(packet[1], 0xaa);
  EXPECT_EQ(packet[2], 30);
  EXPECT_EQ(packet[3], 3);
  EXPECT_EQ(packet[4], 1);
  EXPECT_EQ(packet[8], 10);
  EXPECT_EQ(packet[13], 7);
  EXPECT_EQ(packet[18], 8);
  EXPECT_EQ(packet[28], 0xfc);
  EXPECT_EQ(packet[29], 0);
}
TEST(Serial, FragmentationNoiseConcatenationAndFiltering)
{
  SerialParser parser;
  auto packet = rx_packet(feedback_frame(7, 0, {}, {}));
  const uint8_t noise[] = {1, 2, 3, 4};
  EXPECT_TRUE(parser.feed(noise, 4).empty());
  EXPECT_TRUE(parser.feed(packet.data(), 5).empty());
  auto frames = parser.feed(packet.data() + 5, 11);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].data[0] & 15, 7);
  std::vector<uint8_t> pair(packet.begin(), packet.end());
  pair.insert(pair.end(), packet.begin(), packet.end());
  EXPECT_EQ(parser.feed(pair.data(), pair.size()).size(), 2u);
  packet[2] = 0x48;
  EXPECT_TRUE(parser.feed(packet.data(), packet.size()).empty());
  packet[2] = 0x88;
  EXPECT_TRUE(parser.feed(packet.data(), packet.size()).empty());
  packet[2] = 7;
  EXPECT_TRUE(parser.feed(packet.data(), packet.size()).empty());
  packet = rx_packet({}, 0x12);
  EXPECT_TRUE(parser.feed(packet.data(), packet.size()).empty());
  packet = rx_packet({}, 0x02);
  EXPECT_THROW(parser.feed(packet.data(), packet.size()), std::runtime_error);
}
TEST(Initialization, ReadMappingAndVerifyHardwareWatchdog)
{
  RegisterTransport t;
  auto mapping = initialize_serial(t, 7, 42);
  EXPECT_EQ(mapping.p, 12.5);
  EXPECT_EQ(mapping.v, 30);
  EXPECT_EQ(mapping.t, 10);
  EXPECT_EQ(t.registers.at(9), 4000u);
  for (const auto & f : t.sent) {
    EXPECT_EQ(f.id, 0x7ff);
    EXPECT_EQ(f.data[0], 7);
  }
  EXPECT_EQ(t.sent.back().data[2], 0x33);
  EXPECT_EQ(t.sent.back().data[3], 9);
}
TEST(Initialization, NoEnableOrConfigurationOnUnsafeStartup)
{
  RegisterTransport t;
  t.status = 1;
  EXPECT_THROW(initialize_serial(t, 7, 42), std::runtime_error);
  EXPECT_EQ(t.sent.back().id, 7);
  EXPECT_EQ(t.sent.back().data[7], 0xfd);
  EXPECT_EQ(t.registers.at(9), 0u);
  RegisterTransport wrong_mode;
  wrong_mode.registers[10] = 2;
  EXPECT_THROW(initialize_serial(wrong_mode, 7, 42), std::runtime_error);
  EXPECT_EQ(wrong_mode.sent.size(), 2u);
  RegisterTransport invalid_mapping;
  invalid_mapping.registers[21] = RegisterTransport::bits(std::numeric_limits<float>::quiet_NaN());
  EXPECT_THROW(initialize_serial(invalid_mapping, 7, 42), std::invalid_argument);
}
TEST(Initialization, EchoIsNotEnoughIndependentReadbackRequired)
{
  RegisterTransport t;
  t.apply_write = false;
  EXPECT_THROW(initialize_serial(t, 7, 42), std::runtime_error);
  EXPECT_EQ(t.registers.at(9), 0u);
}
TEST(Controller, StartupRequiresFreshFeedback)
{
  FakeTransport t;
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  EXPECT_THROW(c.enable(0), std::runtime_error);
  t.feedback(0, 8);
  c.tick(0);
  EXPECT_EQ(c.feedback_count(), 0u);
  t.feedback(0);
  c.tick(0.01);
  EXPECT_THROW(c.enable(0.12), std::runtime_error);
  EXPECT_EQ(c.phase(), Phase::Disabled);
}
TEST(Controller, ConfirmationStopResetAndNoOldTarget)
{
  FakeTransport t;
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  enabled(c, t);
  c.command({0.3, 0, 1, 1, 0}, 0.03);
  c.disable(0.04);
  EXPECT_EQ(c.phase(), Phase::Stopping);
  t.feedback(0); c.tick(0.05);
  EXPECT_EQ(c.phase(), Phase::Disabled);
  c.enable(0.06);
  EXPECT_EQ(c.phase(), Phase::Enabling);
  auto neutral = decode_mit(t.sent[t.sent.size() - 2], {});
  EXPECT_EQ(neutral.kp, 0);
  EXPECT_EQ(neutral.kd, 0);
  EXPECT_NEAR(neutral.t, 0, 20.0 / 4095);
  t.feedback(1); c.tick(0.07);
  c.fail("test stop");
  c.tick(0.08);
  EXPECT_EQ(t.sent.back().data[7], 0xfd);
  EXPECT_THROW(c.enable(0.09), std::runtime_error);
  EXPECT_THROW(c.reset(0.09), std::runtime_error);
  t.feedback(0); c.tick(0.10);
  c.reset(0.11);
  EXPECT_EQ(c.phase(), Phase::Disabled);
  EXPECT_TRUE(c.error().empty());
}
TEST(Controller, CommandWatchdogEvenWithHealthyFeedback)
{
  FakeTransport t;
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  enabled(c, t);
  t.feedback(1); c.tick(0.261);
  EXPECT_EQ(c.phase(), Phase::Fault);
  EXPECT_EQ(c.error(), "command publisher timeout");
  EXPECT_EQ(t.sent.back().data[7], 0xfd);
}
TEST(Controller, FeedbackWatchdogWithActiveCommands)
{
  FakeTransport t;
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  enabled(c, t);
  c.command({}, 0.1);
  c.tick(0.121);
  EXPECT_EQ(c.phase(), Phase::Fault);
  EXPECT_EQ(c.error(), "feedback timeout");
}
TEST(Controller, EnableAndDisableAcknowledgementTimeouts)
{
  FakeTransport t;
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  t.feedback(0); c.tick(0);
  c.enable(0.01);
  t.feedback(0); c.tick(0.111);
  EXPECT_EQ(c.error(), "enable acknowledgement timeout");
  t.feedback(0); c.tick(0.12); c.reset(0.13);
  enabled(c, t);
  c.disable(0.04);
  t.feedback(1); c.tick(0.141);
  EXPECT_EQ(c.error(), "disable acknowledgement timeout");
}
TEST(Controller, FaultUnknownStatusAndUnexpectedEnable)
{
  for (uint8_t status : {uint8_t{2}, uint8_t{3}, uint8_t{0xb}}) {
    FakeTransport t;
    Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
    enabled(c, t);
    t.feedback(status); c.tick(0.03);
    EXPECT_EQ(c.phase(), Phase::Fault);
    EXPECT_EQ(t.sent.back().data[7], 0xfd);
  }
  FakeTransport t;
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  t.feedback(1); c.tick(0);
  EXPECT_EQ(c.error(), "unexpected external enable");
}
TEST(Simulation, ClosedLoopAndDisable)
{
  SimTransport t(7, 0, {});
  Controller c(t, 7, 0, {}, {}, 0.25, 0.1);
  c.tick(0); c.tick(0.005);
  c.enable(0.01); c.tick(0.015);
  ASSERT_EQ(c.phase(), Phase::Enabled);
  c.command({0, 0, 0, 0, 0.2}, 0.02);
  c.tick(0.02); c.tick(0.025);
  EXPECT_GT(c.feedback().t, 0.19);
  c.disable(0.03); c.tick(0.035);
  EXPECT_EQ(c.phase(), Phase::Disabled);
}
