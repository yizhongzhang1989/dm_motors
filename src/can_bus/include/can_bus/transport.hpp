#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace can_bus
{
struct Frame
{
  uint32_t id{0};
  std::array<uint8_t, 64> data{};
  uint8_t length{8}, channel{0};
  bool fd{false}, brs{false}, extended{false}, rtr{false}, error{false};
  uint64_t received_steady{0};
};
void validate_frame(const Frame & frame, bool allow_fd = true);
std::array<uint8_t, 30> serial_packet(const Frame & frame);
Frame legacy_serial_frame(const Frame & frame);
class SerialParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, size_t length);
private:
  std::vector<uint8_t> buffer_;
};
class Transport
{
public:
  virtual ~Transport() = default;
  virtual void send(const Frame &) = 0;
  virtual std::vector<Frame> receive() = 0;
};
std::unique_ptr<Transport> make_serial_transport(const std::string & port);
std::unique_ptr<Transport> make_socketcan_transport(const std::string & interface, bool fd);
double steady_seconds();
}
