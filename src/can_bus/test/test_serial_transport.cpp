#include "can_bus/transport.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <future>
#include <pthread.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace
{
class SerialPty : public ::testing::Test
{
protected:
  int master_{-1}, filler_{-1};
  std::unique_ptr<can_bus::Transport> transport_;
  size_t filled_{0};

  void SetUp() override
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(::grantpt(master_), 0);
    ASSERT_EQ(::unlockpt(master_), 0);
    const std::string slave = ::ptsname(master_);
    // Open the test-only filler before the production exclusive TTY lock.
    filler_ = ::open(slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    ASSERT_GE(filler_, 0);
    transport_ = can_bus::make_serial_transport(slave);
  }

  void TearDown() override
  {
    transport_.reset();
    if (filler_ >= 0) {::close(filler_);}
    if (master_ >= 0) {::close(master_);}
  }

  static can_bus::Frame frame(uint32_t id = 1)
  {
    can_bus::Frame result;
    result.id = id;
    std::copy_n("ABCDEFGH", 8, result.data.begin());
    return result;
  }

  void fill_output()
  {
    std::array<uint8_t, 1024> bytes;
    bytes.fill(0x7e);
    // The line discipline transfers asynchronously. Require several successive
    // one-byte EAGAIN results so even a short production packet cannot fit.
    unsigned blocked = 0;
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (blocked < 3 && std::chrono::steady_clock::now() < deadline) {
      auto n = ::write(filler_, bytes.data(), bytes.size());
      if (n < 0 && errno == EAGAIN) {n = ::write(filler_, bytes.data(), 1);}
      if (n > 0) {filled_ += size_t(n); blocked = 0; continue;}
      ASSERT_EQ(n, -1);
      ASSERT_EQ(errno, EAGAIN);
      ++blocked;
      std::this_thread::sleep_for(200us);
    }
    ASSERT_EQ(blocked, 3u);
    ASSERT_GT(filled_, 0u);
  }

  std::vector<uint8_t> drain(size_t expected)
  {
    std::vector<uint8_t> result;
    std::array<uint8_t, 4096> bytes;
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (result.size() < expected && std::chrono::steady_clock::now() < deadline) {
      const auto n = ::read(master_, bytes.data(), bytes.size());
      if (n > 0) {result.insert(result.end(), bytes.begin(), bytes.begin() + n);}
      else if (n < 0 && errno != EAGAIN && errno != EINTR) {break;}
      else {std::this_thread::sleep_for(50us);}
    }
    return result;
  }

  std::string send_error()
  {
    try {transport_->send(frame());}
    catch (const std::runtime_error & error) {return error.what();}
    return {};
  }
};

TEST_F(SerialPty, ProductionPacketAndFeedbackUseTheSamePty)
{
  const auto packet = can_bus::serial_packet(frame());
  transport_->send(frame());
  EXPECT_EQ(drain(packet.size()), std::vector<uint8_t>(packet.begin(), packet.end()));
  const std::array<uint8_t, 16> feedback{{0xaa, 0x11, 8, 1, 0, 0, 0, 8, 7, 6, 5, 4, 3, 2, 1, 0x55}};
  ASSERT_EQ(::write(master_, feedback.data(), feedback.size()), ssize_t(feedback.size()));
  std::vector<can_bus::Frame> received;
  const auto deadline = std::chrono::steady_clock::now() + 100ms;
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    received = transport_->receive();
    std::this_thread::sleep_for(50us);
  }
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].id, 1u);
  EXPECT_EQ(received[0].data[0], 8u);
  EXPECT_EQ(received[0].data[7], 1u);
}

TEST_F(SerialPty, TransientEagainCompletesOnePacketWithoutDroppingOrDuplicatingBytes)
{
  fill_output();
  const auto packet = can_bus::serial_packet(frame());
  std::promise<void> start;
  auto wake = start.get_future();
  auto reader = std::async(std::launch::async, [&] {
      wake.wait();
      std::this_thread::sleep_for(1ms);
      return drain(filled_ + packet.size());
    });
  start.set_value();
  EXPECT_NO_THROW(transport_->send(frame()));
  const auto wire = reader.get();
  ASSERT_EQ(wire.size(), filled_ + packet.size());
  EXPECT_TRUE(std::all_of(wire.begin(), wire.begin() + filled_, [](uint8_t b) {return b == 0x7e;}));
  EXPECT_TRUE(std::equal(packet.begin(), packet.end(), wire.begin() + filled_));
  const auto next = can_bus::serial_packet(frame(2));
  EXPECT_NO_THROW(transport_->send(frame(2)));
  EXPECT_EQ(drain(next.size()), std::vector<uint8_t>(next.begin(), next.end()));
}

TEST_F(SerialPty, PersistentBackpressureFailsWithinOneDeadline)
{
  fill_output();
  const auto started = std::chrono::steady_clock::now();
  const auto error = send_error();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_NE(error.find("serial write timed out"), std::string::npos) << error;
  EXPECT_NE(error.find("bus latched"), std::string::npos) << error;
  EXPECT_GE(elapsed, 4ms);
  EXPECT_LT(elapsed, 100ms);  // Scheduler tolerance, not another transport retry.
}

void interrupted(int) {}

TEST_F(SerialPty, InterruptedPollDoesNotRestartTheAbsoluteDeadline)
{
  fill_output();
  struct sigaction action{}, previous{};
  action.sa_handler = interrupted;
  sigemptyset(&action.sa_mask);
  ASSERT_EQ(::sigaction(SIGUSR1, &action, &previous), 0);
  const auto sender = pthread_self();
  std::atomic<bool> done{false};
  std::thread interrupts([&] {
      while (!done.load()) {
        pthread_kill(sender, SIGUSR1);
        std::this_thread::sleep_for(200us);
      }
    });
  const auto started = std::chrono::steady_clock::now();
  const auto error = send_error();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  done = true;
  interrupts.join();
  ASSERT_EQ(::sigaction(SIGUSR1, &previous, nullptr), 0);
  EXPECT_NE(error.find("serial write timed out"), std::string::npos) << error;
  EXPECT_GE(elapsed, 4ms);
  EXPECT_LT(elapsed, 100ms);
}

TEST_F(SerialPty, DisconnectedPtyStillFailsWithoutRetryingForever)
{
  ::close(master_); master_ = -1;
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(send_error().empty());
  EXPECT_LT(std::chrono::steady_clock::now() - started, 100ms);
}
}  // namespace
