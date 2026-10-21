// Keep os_ros.h first: it enables PCL_NO_PRECOMPILE.
#include "../src/os_sensor_node.h"
#include "ouster_ros/os_ros.h"

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <rcl/rcl.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <mutex>
#include <vector>

namespace {
using namespace std::chrono_literals;
using PacketMsg = ouster_sensor_msgs::msg::PacketMsg;
using namespace ouster::sdk::core;

// Interpose only in this Linux test executable, below the actual sensor node's
// rclcpp Publisher. A paused subscriber does not reliably block DDS on every
// RMW; this gate deterministically reproduces a blocked middleware publish.
struct PublishGate {
  std::mutex mutex;
  std::condition_variable changed;
  bool block_lidar = false;
  bool entered = false;
  std::vector<std::vector<uint8_t>> lidar;
  std::vector<std::vector<uint8_t>> imu;
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    block_lidar = false;
    changed.notify_all();
  }
} gate;

struct ReleaseGate {
  ~ReleaseGate() { gate.release(); }
};

class SensorHarness : public ouster_ros::OusterSensor {
public:
  explicit SensorHarness(const rclcpp::NodeOptions &options)
      : OusterSensor("raw_sensor_backpressure", options) {}
  void open() {
    info = default_sensor_info(LidarMode(2048, 10));
    info.format.pixels_per_column = 128;
    info.format.pixel_shift_by_row.assign(128, 0);
    info.beam_altitude_angles.assign(128, 0.0);
    info.beam_azimuth_angles.assign(128, 0.0);
    create_publishers();
  }
  void close() { on_deactivate(rclcpp_lifecycle::State()); }
  void lidar(const std::vector<uint8_t> &bytes) {
    LidarPacket packet;
    packet.buf = bytes;
    on_lidar_packet_msg(packet);
    EXPECT_EQ(packet.buf, bytes); // No swaps/mutation of caller's storage.
  }
  void imu(const std::vector<uint8_t> &bytes) {
    ImuPacket packet;
    packet.buf = bytes;
    on_imu_packet_msg(packet);
    EXPECT_EQ(packet.buf, bytes);
  }
};

class RawPacketSensorTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    std::lock_guard<std::mutex> lock(gate.mutex);
    gate.entered = false;
    gate.block_lidar = false;
    gate.lidar.clear();
    gate.imu.clear();
  }
  rclcpp::NodeOptions options(int depth = 2) {
    return rclcpp::NodeOptions()
        .use_intra_process_comms(false)
        .parameter_overrides(
            {rclcpp::Parameter("raw_packet_queue_depth", depth)});
  }
};

TEST_F(RawPacketSensorTest, QueueParameterIsValidatedBeforeConnecting) {
  for (int depth : {-1, 0, 65537}) {
    EXPECT_THROW(SensorHarness node(options(depth)), std::invalid_argument);
  }
  SensorHarness node(options());
  EXPECT_EQ(node.get_parameter("raw_packet_queue_depth").as_int(), 2);
  EXPECT_TRUE(node.describe_parameter("raw_packet_queue_depth").read_only);
  EXPECT_FALSE(
      node.set_parameter(rclcpp::Parameter("raw_packet_queue_depth", 4))
          .successful);
}

TEST_F(RawPacketSensorTest,
       BlockedRawLidarCannotBlockReceiveCallbacksOrRawImu) {
  SensorHarness node(options());
  node.open();
  {
    std::lock_guard<std::mutex> lock(gate.mutex);
    gate.block_lidar = true;
  }
  // Declare the release guard after the future: failures unblock before its
  // destructor waits, so a regression fails instead of hanging the runner.
  auto producer = std::async(std::launch::async, [&] {
    node.lidar(std::vector<uint8_t>(24896, 1));
    for (int id = 2; id <= 100; ++id) {
      node.lidar(std::vector<uint8_t>(24896, static_cast<uint8_t>(id)));
    }
    node.imu(std::vector<uint8_t>(48, 7));
  });
  ReleaseGate release;
  {
    std::unique_lock<std::mutex> lock(gate.mutex);
    ASSERT_TRUE(gate.changed.wait_for(lock, 5s, [] { return gate.entered; }));
  }
  ASSERT_EQ(producer.wait_for(2s), std::future_status::ready)
      << "Raw publish stalled the actual sensor receive callback";
  producer.get();
  {
    std::unique_lock<std::mutex> lock(gate.mutex);
    ASSERT_TRUE(
        gate.changed.wait_for(lock, 5s, [] { return !gate.imu.empty(); }));
    EXPECT_TRUE(gate.lidar.empty());
    EXPECT_EQ(gate.imu.front(), std::vector<uint8_t>(48, 7));
  }
  gate.release();
  {
    std::unique_lock<std::mutex> lock(gate.mutex);
    ASSERT_TRUE(
        gate.changed.wait_for(lock, 5s, [] { return !gate.lidar.empty(); }));
    EXPECT_EQ(gate.lidar.front(), std::vector<uint8_t>(24896, 1));
  }
  node.close();
  // Reactivation creates empty queues and accepts the new packet format/data.
  node.open();
  node.imu(std::vector<uint8_t>(48, 9));
  {
    std::unique_lock<std::mutex> lock(gate.mutex);
    ASSERT_TRUE(
        gate.changed.wait_for(lock, 5s, [] { return gate.imu.size() == 2; }));
    EXPECT_EQ(gate.imu.back(), std::vector<uint8_t>(48, 9));
  }
  node.close();
  node.close();
}
} // namespace

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t *publisher,
                                 const void *message,
                                 rmw_publisher_allocation_t *allocation) {
  using Publish = rcl_ret_t (*)(const rcl_publisher_t *, const void *,
                                rmw_publisher_allocation_t *);
  static const auto actual =
      reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rcl_publish"));
  if (!actual)
    return RCL_RET_ERROR;
  const char *topic = rcl_publisher_get_topic_name(publisher);
  const bool lidar = topic && std::strcmp(topic, "/lidar_packets") == 0;
  const bool imu = topic && std::strcmp(topic, "/imu_packets") == 0;
  if (lidar || imu) {
    std::unique_lock<std::mutex> lock(gate.mutex);
    if (lidar) {
      gate.entered = true;
      gate.changed.notify_all();
      gate.changed.wait(lock, [] { return !gate.block_lidar; });
    }
    auto &received = lidar ? gate.lidar : gate.imu;
    received.push_back(static_cast<const PacketMsg *>(message)->buf);
    gate.changed.notify_all();
  }
  return actual(publisher, message, allocation);
}
