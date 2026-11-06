#include "dm_motor_control/runtime.hpp"
#include "dm_motor_control/position_observation_schedule.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>

using namespace dm_motor;
namespace
{
uint32_t bits(float value)
{
  uint32_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}
std::vector<MotorConfig> config()
{
  MotorConfig c; c.name = "axis";
  return {c};
}
bool is_stop(const Frame & f)
{
  return f.length == 8 && f.data[7] == 0xfd &&
    std::all_of(f.data.begin(), f.data.begin() + 7, [](uint8_t b) {return b == 255;});
}
class FakeTransport final : public Transport
{
public:
  struct Read {Frame frame; uint64_t id, submitted, expires;};
  FakeTransport() : sim(config()) {}
  void send(const Frame & frame) override
  {
    normal.push_back(frame);
    order.push_back(is_stop(frame) ? "STOP" : "normal");
    sim.send(frame);
  }
  std::vector<Frame> receive() override
  {
    auto frames = sim.receive();
    if (drop_feedback) {
      frames.erase(std::remove_if(frames.begin(), frames.end(), [](const Frame & f) {
        return !(f.data[1] == 0 && (f.data[2] == 0x33 || f.data[2] == 0x55 || f.data[2] == 0xaa));
      }), frames.end());
    }
    for (auto & f : frames) {if (!f.received_steady) {f.received_steady = clock_ns;}}
    frames.insert(frames.end(), injected.begin(), injected.end()); injected.clear();
    return frames;
  }
  bool activity_available() const override {return activity;}
  bool native_position_observation_supported() const override {++native_support_checks; return supported;}
  bool try_send_native_position_read(const Frame & frame, uint64_t id, uint64_t expires) override
  {
    ++submit_calls;
    if (!permit_submit || outstanding) {return false;}
    outstanding = id; reads.push_back({frame, id, clock_ns, expires}); order.push_back("read");
    return true;  // Publication only; tests independently supply TX evidence and RX.
  }
  std::vector<PositionReadDispatch> take_position_read_dispatches() override
  {
    ++dispatch_takes;
    auto result = dispatches; dispatches.clear(); return result;
  }
  std::vector<PositionReadRemoval> take_position_read_removals() override
  {
    auto result = removals; removals.clear(); return result;
  }
  void cancel_position_read(uint64_t id) override
  {
    if (fail_cancel) {throw std::runtime_error("injected cancellation failure");}
    cancelled.push_back(id);
    if (outstanding == id) {outstanding = 0;}
    dispatches.erase(std::remove_if(dispatches.begin(), dispatches.end(),
      [id](const auto & d) {return d.read_id == id;}), dispatches.end());
    removals.erase(std::remove_if(removals.begin(), removals.end(),
      [id](const auto & d) {return d.read_id == id;}), removals.end());
  }
  void observe(size_t index, uint64_t offset = 100000, bool ambiguous = false)
  {
    const auto & read = reads.at(index);
    dispatches.push_back({read.id, read.submitted + offset, ambiguous});
  }
  void reply(size_t index, float value, uint8_t rid = 0, uint16_t master = 0,
    uint64_t offset = 200000)
  {
    const auto & read = reads.at(index);
    Frame frame = read.frame; frame.id = master;
    if (rid) {frame.data[3] = rid;}
    const auto raw = bits(value);
    for (unsigned i = 0; i < 4; ++i) {frame.data[4 + i] = (raw >> (8 * i)) & 255;}
    frame.received_steady = read.submitted + offset; injected.push_back(frame);
  }
  SimBusTransport sim;
  bool supported{true}, permit_submit{true}, drop_feedback{false}, activity{true};
  bool fail_cancel{false};
  uint64_t clock_ns{0}, outstanding{0};
  mutable unsigned native_support_checks{0};
  unsigned submit_calls{0}, dispatch_takes{0};
  std::vector<Frame> normal, injected;
  std::vector<std::string> order;
  std::vector<Read> reads;
  std::vector<PositionReadDispatch> dispatches;
  std::vector<PositionReadRemoval> removals;
  std::vector<uint64_t> cancelled;
};
class PositionObservationRuntime : public ::testing::Test
{
protected:
  void SetUp() override {create(true);}
  void TearDown() override
  {
    if (bus) {bus->set_position_observation_callback({});}
    bus.reset();
  }
  void create(bool enabled)
  {
    auto fake = std::make_unique<FakeTransport>(); transport = fake.get();
    bus = std::make_unique<Bus>(std::move(fake), config(), true, enabled);
    bus->set_position_observation_callback([this](const PositionObservation & sample) {observations.push_back(sample);});
    for (unsigned i = 0; i < 500 && !bus->ready(); ++i) {tick();}
    ASSERT_TRUE(bus->ready()) << bus->initialization_error();
    tick(); tick();
  }
  void tick(double elapsed = .001)
  {
    now += elapsed; transport->clock_ns = static_cast<uint64_t>(now * 1e9); bus->tick(now);
  }
  void poll(double elapsed = .001)
  {
    now += elapsed; transport->clock_ns = static_cast<uint64_t>(now * 1e9); bus->poll_feedback(now);
  }
  void enable()
  {
    bus->enable("axis", now); tick();
    ASSERT_EQ(bus->snapshot("axis", now).phase, Phase::Enabled);
    torque.torque = .12; bus->command("axis", torque, now); tick();
  }
  void native_setup()
  {
    bus->set_position_observation_callback({});
    bus.reset(); observations.clear(); create(true); enable();
    torque.kp = 100; torque.kd = 1; torque.velocity = .2;
    bus->command("axis", torque, now);
  }
  bool native_request(uint64_t id = 1)
  {
    return bus->request_position_observation("axis", id, now,
      PositionObservationContext{3, 5, 7, "source"}, [this] {return task_alive;},
      [this] {return lineage_alive;});
  }
  std::unique_ptr<Bus> bus;
  FakeTransport * transport{nullptr};
  double now{0};
  Target torque;
  bool task_alive{true}, lineage_alive{true};
  std::vector<PositionObservation> observations;
};
}

TEST_F(PositionObservationRuntime, DefaultOffLeavesTransportAndControlUntouched)
{
  bus->set_position_observation_callback({}); bus.reset(); observations.clear(); create(false); enable();
  EXPECT_FALSE(native_request());
  tick(); poll();
  EXPECT_EQ(transport->native_support_checks, 0u);
  EXPECT_EQ(transport->submit_calls, 0u);
  EXPECT_EQ(transport->dispatch_takes, 0u);
  EXPECT_TRUE(transport->cancelled.empty());
  EXPECT_TRUE(observations.empty());
  EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Enabled);
}

TEST_F(PositionObservationRuntime, WrongRidMasterAndPreDispatchRxCannotCompletePoint)
{
  native_setup(); ASSERT_TRUE(native_request()); tick(); transport->observe(0);
  transport->reply(0, 10.f, 81); transport->reply(0, 11.f, 80, 7);
  transport->reply(0, 12.f, 80, 0, 50000);  // Acquired before observed TX at +100us.
  poll(); EXPECT_TRUE(observations.empty()); EXPECT_EQ(transport->reads.size(), 1u);
  transport->reply(0, .125f); tick();
  ASSERT_EQ(transport->reads.size(), 1u);
  ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Complete);
  EXPECT_DOUBLE_EQ(observations[0].point.radians, .125);
}

TEST_F(PositionObservationRuntime, DistinctObservedTransactionsAreAmbiguous)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  transport->observe(0); transport->observe(0, 150000); transport->reply(0, .125f); poll();
  ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Ambiguous);
  EXPECT_TRUE(observations[0].session_locked); EXPECT_EQ(transport->reads.size(), 1u);
  EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Enabled);
}

TEST_F(PositionObservationRuntime, NativeObservationAcceptsNonzeroGainsAndCompletesOnly80)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  ASSERT_EQ(transport->reads.size(), 1u);
  EXPECT_EQ(transport->reads[0].frame.data[3], 80);
  ASSERT_GE(transport->order.size(), 2u);
  EXPECT_EQ(transport->order[transport->order.size()-2], "normal");
  EXPECT_EQ(transport->order.back(), "read");
  // A new target in the same live source does not cancel/restart the read.
  torque.position = .001; torque.kp = 50; bus->command("axis", torque, now);
  transport->observe(0); transport->reply(0, .125f); poll();
  ASSERT_EQ(observations.size(), 1u); const auto & result = observations.front();
  EXPECT_EQ(result.state, PositionObservationState::Complete); EXPECT_TRUE(result.point.valid);
  EXPECT_DOUBLE_EQ(result.point.radians, .125); EXPECT_LT(result.point.tx_steady, result.point.rx_steady);
  EXPECT_EQ(result.context.observation_epoch, 3u); EXPECT_EQ(result.context.bus_epoch, 5u);
  EXPECT_EQ(result.context.task_id, 7u); EXPECT_EQ(result.context.source_id, "source");
  EXPECT_DOUBLE_EQ(result.mapping.p, config()[0].mapping.p);
  EXPECT_FALSE(result.session_locked);
  tick(); EXPECT_EQ(transport->reads.size(), 1u);  // No automatic new request.
  EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Enabled);
}

TEST_F(PositionObservationRuntime, NativeObservationExcludesAnotherReadAndMaintenance)
{
  native_setup(); ASSERT_TRUE(native_request());
  EXPECT_FALSE(native_request(2));
  const auto operation = bus->read_register("axis", 80, now);
  EXPECT_EQ(bus->operation(operation).state, OperationState::Rejected);
  bus->cancel_position_observation("axis", "source changed before publish");
  ASSERT_EQ(observations.size(), 1u); EXPECT_FALSE(observations[0].session_locked);
  EXPECT_TRUE(native_request(3));
}

TEST_F(PositionObservationRuntime, CancellationFailureCannotPublishACompleteObservation)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  transport->fail_cancel = true;
  transport->observe(0); transport->reply(0, .125f); poll();
  ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Ambiguous);
  EXPECT_FALSE(observations[0].point.valid);
  EXPECT_TRUE(observations[0].session_locked); EXPECT_TRUE(bus->position_observation_locked());
  EXPECT_FALSE(native_request(2));
  EXPECT_NO_THROW(bus->command("axis", torque, now));
  EXPECT_FALSE(bus->snapshot("axis", now).host_fault);
}

TEST_F(PositionObservationRuntime, NativeContextRequiresValidSourceAndPublishedCancellationLocksAcrossEpochs)
{
  native_setup(); task_alive = false; EXPECT_FALSE(native_request());
  task_alive = true; ASSERT_TRUE(native_request()); tick();
  task_alive = false; poll();
  ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
  EXPECT_TRUE(observations[0].session_locked); EXPECT_TRUE(bus->position_observation_locked());
  task_alive = true;
  EXPECT_FALSE(bus->request_position_observation("axis", 2, now,
    PositionObservationContext{4, 5, 8, "new-source"}, [] {return true;}, [] {return true;}));
  transport->observe(0); transport->reply(0, .25f); tick();
  EXPECT_EQ(observations.size(), 1u); EXPECT_EQ(transport->reads.size(), 1u);
  EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Enabled);
}

TEST_F(PositionObservationRuntime, NativeNeverPublishedBusyDoesNotLockOrAlterMotor)
{
  native_setup(); transport->permit_submit = false; ASSERT_TRUE(native_request()); tick();
  EXPECT_TRUE(transport->reads.empty()); EXPECT_FALSE(native_request(2));
  tick(.020);
  ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Timeout);
  EXPECT_FALSE(observations[0].session_locked); EXPECT_FALSE(bus->position_observation_locked());
  EXPECT_FALSE(bus->snapshot("axis", now).host_fault);
  transport->permit_submit = true; ASSERT_TRUE(native_request(3)); tick();
  ASSERT_EQ(transport->reads.size(), 1u);
}

TEST_F(PositionObservationRuntime, NativeMissingTxTimesOutWithoutRetryAndStopKeepsLock)
{
  native_setup(); ASSERT_TRUE(native_request()); tick(); transport->reply(0, .25f);
  tick(.020); ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Timeout);
  EXPECT_TRUE(observations[0].session_locked); EXPECT_FALSE(native_request(2));
  bus->disable("axis", now); tick(); tick();
  EXPECT_TRUE(bus->position_observation_locked()); EXPECT_FALSE(native_request(3));
  EXPECT_EQ(transport->reads.size(), 1u);
  EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Disabled);
}

TEST_F(PositionObservationRuntime, NativeRxBeforeTxCallbackAndFutureAcquisitionRemainCausal)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  transport->reply(0, .25f, 80, 0, 5000000); poll(.001);
  EXPECT_TRUE(observations.empty()); transport->observe(0); poll(.001);
  EXPECT_TRUE(observations.empty()); poll(.004);
  ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Complete);
  EXPECT_LE(observations[0].point.rx_steady, static_cast<uint64_t>(now * 1e9));
  EXPECT_EQ(transport->reads.size(), 1u);
}

TEST_F(PositionObservationRuntime, NativeReplyCannotRefreshCompactOrReanchorPosition)
{
  native_setup(); transport->drop_feedback = true; poll();
  const auto before = bus->snapshot("axis", now);
  ASSERT_TRUE(native_request()); tick(); transport->observe(0); transport->reply(0, 7.5f); poll();
  ASSERT_EQ(observations.size(), 1u); EXPECT_TRUE(observations[0].point.valid);
  const auto after = bus->snapshot("axis", now);
  EXPECT_EQ(after.feedback_count, before.feedback_count);
  EXPECT_GT(after.feedback_age, before.feedback_age);
  EXPECT_DOUBLE_EQ(after.absolute_position, before.absolute_position);
  EXPECT_EQ(after.phase, Phase::Enabled);
}

TEST_F(PositionObservationRuntime, NativeAmbiguityLocksWithoutAffectingOrdinaryControl)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  transport->observe(0); transport->reply(0, .125f); transport->reply(0, .25f, 80, 0, 300000);
  poll(); ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Ambiguous);
  EXPECT_TRUE(observations[0].session_locked);
  EXPECT_NO_THROW(bus->command("axis", torque, now)); tick();
  EXPECT_EQ(transport->reads.size(), 1u); EXPECT_FALSE(bus->snapshot("axis", now).host_fault);
}

TEST_F(PositionObservationRuntime, NormalStopBeforePublicationCancelsWithoutLockOrRead)
{
  native_setup(); ASSERT_TRUE(native_request());
  const auto stops = transport->normal.size();
  bus->disable("axis", now);
  ASSERT_EQ(transport->normal.size(), stops + 1); EXPECT_TRUE(is_stop(transport->normal.back()));
  ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
  EXPECT_FALSE(observations[0].point.valid); EXPECT_FALSE(observations[0].session_locked);
  EXPECT_FALSE(bus->position_observation_retiring()); EXPECT_FALSE(bus->position_observation_locked());
  EXPECT_TRUE(transport->reads.empty());
  bus->disable("axis", now); EXPECT_EQ(observations.size(), 1u);
  tick(); enable(); ASSERT_TRUE(native_request(2)); tick();
  EXPECT_EQ(transport->reads.size(), 1u);
}

TEST_F(PositionObservationRuntime, NormalStopRetiresPublishedReadThenNewEpochCanComplete)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  const auto read_id = transport->outstanding;
  const auto deadline = transport->reads[0].expires;
  const auto stops = transport->normal.size();
  bus->disable("axis", now); task_alive = false;
  ASSERT_EQ(transport->normal.size(), stops + 1); EXPECT_TRUE(is_stop(transport->normal.back()));
  EXPECT_TRUE(bus->position_observation_retiring()); EXPECT_EQ(transport->outstanding, read_id);
  EXPECT_TRUE(observations.empty()); EXPECT_FALSE(native_request(2));
  // STOP does not wait for the broker TX callback. The old wire response may
  // complete while already disabled, but it is never a usable observation.
  tick(); EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Disabled);
  bus->disable("axis", now); EXPECT_EQ(transport->reads[0].expires, deadline);
  transport->observe(0); transport->reply(0, .25f); poll();
  ASSERT_EQ(observations.size(), 1u); const auto retired = observations[0];
  EXPECT_EQ(retired.state, PositionObservationState::Cancelled); EXPECT_FALSE(retired.point.valid);
  EXPECT_FALSE(retired.session_locked); EXPECT_EQ(retired.context.observation_epoch, 3u);
  EXPECT_EQ(retired.context.source_id, "source"); EXPECT_EQ(retired.id, 1u);
  EXPECT_EQ(retired.point.raw, bits(.25f)); EXPECT_DOUBLE_EQ(retired.point.radians, .25);
  EXPECT_GT(retired.point.rx_steady, retired.point.tx_steady);
  EXPECT_FALSE(bus->position_observation_retiring()); EXPECT_FALSE(bus->position_observation_locked());
  EXPECT_EQ(transport->outstanding, 0u); EXPECT_EQ(transport->reads.size(), 1u);
  enable();
  ASSERT_TRUE(bus->request_position_observation("axis", 3, now,
    PositionObservationContext{4, 5, 8, "new-source"}, [] {return true;}, [] {return true;}));
  tick(); transport->observe(1); transport->reply(1, .5f); poll();
  ASSERT_EQ(observations.size(), 2u); EXPECT_EQ(observations[1].state, PositionObservationState::Complete);
  EXPECT_TRUE(observations[1].point.valid); EXPECT_EQ(observations[1].context.observation_epoch, 4u);
  EXPECT_EQ(observations[1].context.source_id, "new-source");
  for (const auto & read : transport->reads) {EXPECT_EQ(read.frame.data[3], 80);}
}

TEST_F(PositionObservationRuntime, ExplicitSourceRetirementIsIdempotentAndDoesNotStopOrdinaryControl)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  const auto stopped_before = std::count_if(transport->normal.begin(), transport->normal.end(), is_stop);
  const auto deadline = transport->reads[0].expires;
  task_alive = false;
  bus->retire_position_observation("axis", "first source handover");
  bus->retire_position_observation("axis", "second handover must not extend or replace");
  EXPECT_TRUE(bus->position_observation_retiring()); EXPECT_FALSE(native_request(2));
  EXPECT_NO_THROW(bus->command("axis", torque, now)); tick(.005);
  EXPECT_EQ(std::count_if(transport->normal.begin(), transport->normal.end(), is_stop), stopped_before);
  EXPECT_EQ(transport->reads.size(), 1u); EXPECT_EQ(transport->reads[0].expires, deadline);
  transport->observe(0); transport->reply(0, .125f); poll();
  ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
  EXPECT_EQ(observations[0].message.find("first source handover"), 0u);
  EXPECT_FALSE(observations[0].point.valid); EXPECT_FALSE(bus->position_observation_locked());
  bus->retire_position_observation("axis", "already closed"); poll();
  EXPECT_EQ(observations.size(), 1u); EXPECT_EQ(bus->snapshot("axis", now).phase, Phase::Enabled);
}

TEST_F(PositionObservationRuntime, RetiringReadKeepsReceiverBeforeTxAndFutureAcquisitionCausal)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  bus->retire_position_observation("axis", "source ended"); task_alive = false;
  transport->drop_feedback = true;
  const auto before = bus->snapshot("axis", now);
  transport->reply(0, .25f, 80, 0, 5000000); poll(.001);
  EXPECT_TRUE(observations.empty()); transport->observe(0); poll(.001);
  EXPECT_TRUE(observations.empty()); poll(.004);
  ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
  EXPECT_FALSE(observations[0].point.valid); EXPECT_FALSE(observations[0].session_locked);
  EXPECT_LE(observations[0].point.rx_steady, static_cast<uint64_t>(now * 1e9));
  const auto after = bus->snapshot("axis", now);
  EXPECT_EQ(after.feedback_count, before.feedback_count);
  EXPECT_DOUBLE_EQ(after.absolute_position, before.absolute_position);
}

TEST_F(PositionObservationRuntime, RetiringMissingWitnessOrLatePollStillLocksAtOriginalDeadline)
{
  for (unsigned variant = 0; variant < 3; ++variant) {
    SCOPED_TRACE(variant); task_alive = true; native_setup(); ASSERT_TRUE(native_request()); tick();
    const auto deadline = transport->reads[0].expires;
    bus->retire_position_observation("axis", "normal end"); task_alive = false;
    if (variant != 0) {transport->observe(0);}  // RX without TX / TX without RX / both, late poll.
    if (variant != 1) {transport->reply(0, .25f);}
    bus->retire_position_observation("axis", "no extension");
    EXPECT_EQ(transport->reads[0].expires, deadline);
    poll(.020);
    ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Timeout);
    EXPECT_TRUE(observations[0].session_locked); EXPECT_TRUE(bus->position_observation_locked());
    bus->disable("axis", now); task_alive = true; tick();
    EXPECT_FALSE(native_request(2)); EXPECT_EQ(transport->reads.size(), 1u);
    transport->observe(0); transport->reply(0, .5f); poll();
    EXPECT_EQ(observations.size(), 1u); EXPECT_TRUE(bus->position_observation_locked());
  }
}

TEST_F(PositionObservationRuntime, RetiringAmbiguousOrNonfiniteWireEvidenceStillLocks)
{
  for (unsigned variant = 0; variant < 3; ++variant) {
    SCOPED_TRACE(variant); task_alive = true; native_setup(); ASSERT_TRUE(native_request()); tick();
    bus->retire_position_observation("axis", "normal end"); task_alive = false;
    transport->observe(0);
    if (variant == 0) {transport->observe(0, 150000, true);}
    transport->reply(0, variant == 2 ? std::numeric_limits<float>::quiet_NaN() : .25f);
    if (variant == 1) {transport->reply(0, .5f, 80, 0, 300000);}
    poll(); ASSERT_EQ(observations.size(), 1u);
    EXPECT_EQ(observations[0].state, PositionObservationState::Ambiguous);
    EXPECT_TRUE(bus->position_observation_locked()); EXPECT_FALSE(observations[0].point.valid);
    EXPECT_NO_THROW(bus->command("axis", torque, now)); tick();
    EXPECT_EQ(transport->reads.size(), 1u); EXPECT_FALSE(bus->snapshot("axis", now).host_fault);
  }
}

TEST_F(PositionObservationRuntime, RetiringLineageLossFaultAndHardCancelNeverBecomeCleanRetirement)
{
  for (unsigned variant = 0; variant < 4; ++variant) {
    SCOPED_TRACE(variant); task_alive = true; lineage_alive = true;
    native_setup(); ASSERT_TRUE(native_request()); tick();
    bus->retire_position_observation("axis", "normal end"); task_alive = false;
    if (variant == 0) {lineage_alive = false;}  // Broker epoch/route predicate.
    if (variant == 1) {transport->activity = false;}
    if (variant == 2) {bus->fail("axis", "real fault");}
    if (variant == 3) {bus->cancel_position_observation("axis", "hard boundary");}
    transport->observe(0); transport->reply(0, .25f); poll();
    ASSERT_EQ(observations.size(), 1u); EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
    EXPECT_TRUE(observations[0].session_locked); EXPECT_FALSE(observations[0].point.valid);
    EXPECT_TRUE(bus->position_observation_locked()); EXPECT_FALSE(bus->position_observation_retiring());
    EXPECT_EQ(transport->outstanding, 0u);
    lineage_alive = true; transport->activity = true; bus->disable("axis", now); tick();
    EXPECT_TRUE(bus->position_observation_locked()); EXPECT_FALSE(native_request(2));
  }
}

TEST_F(PositionObservationRuntime, RetiringPreservesMaintenanceExclusionAndCallbackFailureLock)
{
  native_setup(); ASSERT_TRUE(native_request()); tick();
  bus->disable("axis", now); tick();
  const auto read = bus->read_register("axis", 80, now);
  EXPECT_EQ(bus->operation(read).state, OperationState::Rejected);
  EXPECT_FALSE(native_request(2));
  unsigned callbacks = 0;
  bus->set_position_observation_callback([&](const PositionObservation & sample) {
    ++callbacks; EXPECT_EQ(sample.state, PositionObservationState::Cancelled);
    EXPECT_FALSE(sample.point.valid); throw std::runtime_error("subscriber failure");
  });
  transport->observe(0); transport->reply(0, .25f); poll();
  EXPECT_EQ(callbacks, 1u); EXPECT_TRUE(bus->position_observation_locked());
  EXPECT_FALSE(bus->position_observation_retiring()); EXPECT_EQ(transport->reads.size(), 1u);
  bus->disable("axis", now); EXPECT_EQ(callbacks, 1u);
}

TEST_F(PositionObservationRuntime, RetirementDoesNotWaiveCompactFreshness)
{
  native_setup(); transport->drop_feedback = true;
  poll(config()[0].feedback_timeout - .005);
  ASSERT_TRUE(native_request()); tick();
  bus->retire_position_observation("axis", "normal source change"); task_alive = false;
  transport->observe(0); transport->reply(0, .25f); poll(.006);
  ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
  EXPECT_TRUE(observations[0].session_locked); EXPECT_FALSE(observations[0].point.valid);
  EXPECT_TRUE(bus->position_observation_locked());
}

TEST(PositionObservationSchedule, SameIdentityKeepsEpochAndLateSlotsNeverBurst)
{
  PositionObservationSchedule schedule;
  const PositionObservationIdentity identity{"stream", "lease", 0, 0, 1};
  schedule.bind(identity, 1000000000); EXPECT_EQ(schedule.epoch(), 1u);
  EXPECT_TRUE(schedule.take_slot(1000000000)); EXPECT_FALSE(schedule.take_slot(1000000000));
  schedule.bind(identity, 1001000000); EXPECT_EQ(schedule.epoch(), 1u);
  EXPECT_FALSE(schedule.take_slot(1007000000));
  EXPECT_TRUE(schedule.take_slot(1032000000)); EXPECT_EQ(schedule.missed_slots(), 3u);
  EXPECT_FALSE(schedule.take_slot(1032000000)); schedule.missed();
  EXPECT_EQ(schedule.missed_slots(), 4u); schedule.completed(); EXPECT_EQ(schedule.completed_count(), 1u);
  schedule.clear(); EXPECT_FALSE(schedule.take_slot(1040000000));
  schedule.bind(identity, 1040000000); EXPECT_EQ(schedule.epoch(), 2u);
  EXPECT_EQ(schedule.missed_slots(), 4u); EXPECT_EQ(schedule.completed_count(), 1u);
}

TEST_F(PositionObservationRuntime, NativeCloseDeadlineSkipsSlotWithoutSubmitting)
{
  native_setup();
  PositionObservationSchedule schedule;
  const uint64_t start = static_cast<uint64_t>(now * 1e9);
  schedule.bind({"source", "lease", 7, 1, 5}, start);
  if (schedule.take_slot(start, start + 20000000)) {EXPECT_TRUE(native_request());}
  EXPECT_EQ(schedule.missed_slots(), 1u);
  tick();
  EXPECT_EQ(transport->submit_calls, 0u);
  EXPECT_TRUE(transport->reads.empty()); EXPECT_TRUE(observations.empty());
  EXPECT_FALSE(bus->position_observation_locked());
  EXPECT_FALSE(schedule.take_slot(start + 8000000, start + 8000000));
  EXPECT_EQ(schedule.missed_slots(), 2u);
}

TEST(PositionObservationSchedule, SameIdentityDeadlineExtensionResumesOriginalCadence)
{
  PositionObservationSchedule schedule;
  const PositionObservationIdentity identity{"stream", "lease", 0, 0, 1};
  const uint64_t start = 1000000000;
  schedule.bind(identity, start);
  EXPECT_FALSE(schedule.take_slot(start, start + 20000000));
  EXPECT_EQ(schedule.missed_slots(), 1u);
  schedule.bind(identity, start + 3000000);  // Same source renewed; identity is unchanged.
  EXPECT_EQ(schedule.epoch(), 1u);
  EXPECT_FALSE(schedule.take_slot(start + 3000000, start + 100000000));
  EXPECT_FALSE(schedule.take_slot(start + 7999999, start + 100000000));
  EXPECT_TRUE(schedule.take_slot(start + 8000000, start + 100000000));
  EXPECT_FALSE(schedule.take_slot(start + 8000000, start + 100000000));
  // A late close-deadline call consumes missed old slots plus its own slot.
  EXPECT_FALSE(schedule.take_slot(start + 32000000, start + 52000000));
  EXPECT_EQ(schedule.missed_slots(), 4u);
  EXPECT_FALSE(schedule.take_slot(start + 39000000, start + 100000000));
  EXPECT_TRUE(schedule.take_slot(start + 40000000, start + 100000000));
  EXPECT_EQ(schedule.epoch(), 1u); EXPECT_EQ(schedule.missed_slots(), 4u);
}

TEST(PositionObservationSchedule, SourceLeaseTaskAndBusChangesRebindButEmptySourceCannotActivate)
{
  PositionObservationSchedule schedule;
  PositionObservationIdentity identity{"stream", "lease", 0, 0, 1};
  schedule.bind(identity, 1); identity.lease = "new"; schedule.bind(identity, 2);
  EXPECT_EQ(schedule.epoch(), 2u); identity.task = 2; schedule.bind(identity, 3);
  EXPECT_EQ(schedule.epoch(), 3u); identity.task_instance = 99; schedule.bind(identity, 4);
  EXPECT_EQ(schedule.epoch(), 4u); identity.source = "other"; schedule.bind(identity, 5);
  EXPECT_EQ(schedule.epoch(), 5u); identity.bus_epoch = 2; schedule.bind(identity, 6);
  EXPECT_EQ(schedule.epoch(), 6u); identity.source.clear(); schedule.bind(identity, 7);
  EXPECT_FALSE(schedule.active()); EXPECT_FALSE(schedule.take_slot(10));
}

TEST_F(PositionObservationRuntime, BrokerRemovedNativeReadCancelsWithoutWireThenNewEpochCompletes)
{
  native_setup(); ASSERT_TRUE(native_request()); tick(); const auto read = transport->reads[0];
  bus->disable("axis", now); task_alive = false;
  ASSERT_TRUE(is_stop(transport->normal.back())); EXPECT_TRUE(observations.empty());
  transport->removals.push_back({read.id, read.submitted + 100000, false});
  transport->removals.push_back({read.id, read.submitted + 100000, false});
  poll(); ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled);
  EXPECT_FALSE(observations[0].point.valid); EXPECT_FALSE(observations[0].session_locked);
  EXPECT_EQ(observations[0].point.tx_steady, 0u); EXPECT_EQ(observations[0].point.rx_steady, 0u);
  EXPECT_EQ(observations[0].context.observation_epoch, 3u);
  EXPECT_FALSE(bus->position_observation_locked()); EXPECT_FALSE(bus->position_observation_retiring());
  EXPECT_EQ(transport->outstanding, 0u); EXPECT_EQ(transport->reads.size(), 1u);
  tick(); enable(); task_alive = true;
  ASSERT_TRUE(bus->request_position_observation("axis", 2, now,
    PositionObservationContext{4, 5, 8, "new-source"}, [] {return true;}, [] {return true;}));
  tick(); transport->observe(1); transport->reply(1, .5f); poll();
  ASSERT_EQ(observations.size(), 2u); EXPECT_EQ(observations[1].state, PositionObservationState::Complete);
  EXPECT_EQ(observations[1].context.observation_epoch, 4u); EXPECT_TRUE(observations[1].point.valid);
}

TEST_F(PositionObservationRuntime, BrokerRemovalNeedsRetirementNoWireAndValidTimingLineage)
{
  for (unsigned variant = 0; variant < 7; ++variant) {
    SCOPED_TRACE(variant); task_alive = true; lineage_alive = true;
    native_setup(); ASSERT_TRUE(native_request()); tick(); const auto read = transport->reads[0];
    if (variant != 0) {bus->disable("axis", now); task_alive = false;}
    if (variant == 1) {transport->observe(0);}
    if (variant == 2) {transport->reply(0, .25f);}
    if (variant == 6) {lineage_alive = false;}
    const auto stamp = variant == 3 ? read.submitted - 1 : variant == 4 ? read.expires : read.submitted + 100000;
    transport->removals.push_back({read.id, stamp, variant == 5}); poll();
    ASSERT_EQ(observations.size(), 1u); EXPECT_FALSE(observations[0].point.valid);
    EXPECT_TRUE(observations[0].session_locked); EXPECT_TRUE(bus->position_observation_locked());
    EXPECT_FALSE(native_request(2));
  }
}

TEST_F(PositionObservationRuntime, BrokerRemovalNeverClearsTimeoutOrHardCancelOrWrongRead)
{
  for (unsigned variant = 0; variant < 3; ++variant) {
    SCOPED_TRACE(variant); task_alive = true; lineage_alive = true;
    native_setup(); ASSERT_TRUE(native_request()); tick(); const auto read = transport->reads[0];
    bus->disable("axis", now); task_alive = false;
    if (variant == 0) {bus->cancel_position_observation("axis", "hard cancel");}
    if (variant == 1) {poll(.021);}
    transport->removals.push_back({read.id + (variant == 2 ? 100 : 0), read.submitted + 100000, false});
    poll(variant == 2 ? .021 : .001);
    ASSERT_EQ(observations.size(), 1u); EXPECT_TRUE(observations[0].session_locked);
    if (variant != 0) {EXPECT_EQ(observations[0].state, PositionObservationState::Timeout);}
    tick(); enable(); task_alive = true; EXPECT_FALSE(native_request(2));
    EXPECT_TRUE(bus->position_observation_locked()); EXPECT_EQ(observations.size(), 1u);
  }
}

TEST_F(PositionObservationRuntime, BrokerRemovalWaitsForItsAcquisitionTimeWithoutExtendingDeadline)
{
  native_setup(); ASSERT_TRUE(native_request()); tick(); const auto read = transport->reads[0];
  bus->disable("axis", now); task_alive = false;
  transport->removals.push_back({read.id, read.submitted + 5000000, false});
  poll(.001); EXPECT_TRUE(observations.empty()); EXPECT_FALSE(native_request(2));
  poll(.005); ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations[0].state, PositionObservationState::Cancelled); EXPECT_FALSE(observations[0].session_locked);
  EXPECT_EQ(transport->reads[0].expires, read.expires);
}
