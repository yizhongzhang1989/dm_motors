#pragma once

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace common
{
// Linux abstract sockets release ownership even after a forced process exit.
class ProcessLock
{
public:
  ProcessLock() = default;
  ProcessLock(const ProcessLock &) = delete;
  ProcessLock & operator=(const ProcessLock &) = delete;
  ~ProcessLock() {if (fd_ >= 0) {close(fd_);}}
  void acquire(const std::string & resource)
  {
    if (fd_ >= 0) {throw std::logic_error("Process lock already acquired");}
    const auto key = "ros_resource_" + std::to_string(std::hash<std::string>{}(resource));
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path + 1, key.data(), key.size());
    fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd_ < 0 || bind(fd_, reinterpret_cast<sockaddr *>(&address),
        offsetof(sockaddr_un, sun_path) + 1 + key.size()) < 0)
    {
      const std::string reason = std::strerror(errno);
      if (fd_ >= 0) {close(fd_); fd_ = -1;}
      throw std::runtime_error("Resource already running or unavailable: " + resource + ": " + reason);
    }
  }
private:
  int fd_{-1};
};
}  // namespace common
