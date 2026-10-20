#include "common/config.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <unistd.h>

using common::Config;

TEST(Config, ReadsInstalledNodeDefaultsWithoutMutatingTree)
{
  Config config;
  EXPECT_EQ(config.get<int>("packages.can_bus.nodes.dashboard.port"), 8080);
  EXPECT_DOUBLE_EQ(config.get<double>("packages.dm_motor_control.nodes.motor_node.command_timeout"), 0.25);
  EXPECT_EQ(config.get<int>("packages.can_bus.nodes.dashboard.port"), 8080);
  EXPECT_THROW(config.get<int>("packages.missing"), std::runtime_error);
  EXPECT_THROW(config.get<int>("packages.dm_motor_control.nodes.motor_node.host_protection_enabled"), std::runtime_error);
}

TEST(Config, AcceptsGenericPackagesWithoutMotorSections)
{
  char path[] = "/tmp/common_config_XXXXXX";
  const int descriptor = mkstemp(path);
  ASSERT_GE(descriptor, 0);
  close(descriptor);
  {
    std::ofstream file(path);
    file << "version: 2\npackages:\n  camera_driver:\n    nodes:\n      camera:\n        fps: 60\n"
      "    launches:\n      preview.launch.py:\n        parameters:\n          fps: 10\n";
  }
  EXPECT_EQ(Config(path).get<int>("packages.camera_driver.nodes.camera.fps"), 60);
  EXPECT_EQ(Config(path).get<int>(std::vector<std::string>{
      "packages", "camera_driver", "launches", "preview.launch.py", "parameters", "fps"}), 10);
  {
    std::ofstream file(path);
    file << "version: 2\nversion: 2\n";
  }
  EXPECT_THROW(Config{path}, std::runtime_error);
  {
    std::ofstream file(path);
    file << "version: 2\npackages:\n  camera_driver:\n    gain: .nan\n";
  }
  EXPECT_THROW(Config{path}, std::runtime_error);
  unlink(path);
  EXPECT_THROW(Config{path}, std::runtime_error);
}
