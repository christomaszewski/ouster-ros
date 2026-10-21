// Keep os_ros.h first: it enables PCL_NO_PRECOMPILE.
#include "ouster_ros/os_ros.h"

#include <class_loader/class_loader.hpp>
#include <gtest/gtest.h>
#include <ouster/impl/packet_writer.h>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "ouster_ros/raw_packet_qos.h"
#include "ouster_sensor_msgs/msg/packet_msg.hpp"

namespace {

using namespace std::chrono_literals;
using namespace ouster::sdk::core;
using PacketMsg = ouster_sensor_msgs::msg::PacketMsg;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using ImuMsg = sensor_msgs::msg::Imu;

constexpr uint64_t START_NS = 100000000000ULL;
constexpr uint64_t IMU_PERIOD_NS = 10000000ULL;
constexpr size_t HEIGHT = 128;

uint32_t expected_range(size_t frame, size_t ring, size_t column) {
  return 1000 + frame * 10000 + ring * 10 + column;
}

class RawPacketQosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr
  parameters(const std::vector<rclcpp::Parameter> &overrides = {}) {
    return std::make_shared<rclcpp::Node>(
        "raw_packet_qos_parameters",
        rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

TEST_F(RawPacketQosTest, DefaultProfileIsExplicitAndStartupOnly) {
  auto node = parameters();
  ouster_ros::declare_raw_packet_qos_parameters(*node);
  const auto profile = ouster_ros::raw_packet_qos(*node).get_rmw_qos_profile();
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 1024u);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_TRUE(node->describe_parameter("raw_packet_qos_depth").read_only);
  EXPECT_TRUE(node->describe_parameter("raw_packet_qos_reliability").read_only);
  EXPECT_FALSE(
      node->set_parameter(rclcpp::Parameter("raw_packet_qos_depth", 32))
          .successful);
  EXPECT_FALSE(
      node->set_parameter(
              rclcpp::Parameter("raw_packet_qos_reliability", "best_effort"))
          .successful);
}

TEST_F(RawPacketQosTest, BestEffortAndBoundedDepthCanBeSelectedExplicitly) {
  auto node = parameters(
      {rclcpp::Parameter("raw_packet_qos_depth", 32),
       rclcpp::Parameter("raw_packet_qos_reliability", "best_effort")});
  ouster_ros::declare_raw_packet_qos_parameters(*node);
  const auto profile = ouster_ros::raw_packet_qos(*node).get_rmw_qos_profile();
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 32u);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST_F(RawPacketQosTest, DepthBoundariesAreAccepted) {
  for (const int depth : {1, 65536}) {
    auto node = parameters({rclcpp::Parameter("raw_packet_qos_depth", depth)});
    ASSERT_NO_THROW(ouster_ros::declare_raw_packet_qos_parameters(*node));
    EXPECT_EQ(ouster_ros::raw_packet_qos(*node).get_rmw_qos_profile().depth,
              static_cast<size_t>(depth));
  }
}

TEST_F(RawPacketQosTest, InvalidDepthIsRejectedBeforeAnyEndpointExists) {
  for (const int depth : {-1, 0, 65537}) {
    auto node = parameters({rclcpp::Parameter("raw_packet_qos_depth", depth)});
    EXPECT_ANY_THROW(ouster_ros::declare_raw_packet_qos_parameters(*node));
  }
}

TEST_F(RawPacketQosTest, InvalidReliabilityIsRejectedBeforeAnyEndpointExists) {
  for (const auto *reliability :
       {"", "reliabel", "system_default", "RELIABLE"}) {
    auto node = parameters(
        {rclcpp::Parameter("raw_packet_qos_reliability", reliability)});
    EXPECT_ANY_THROW(ouster_ros::declare_raw_packet_qos_parameters(*node));
  }
}

class RawPacketQosComponentTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Keep the plugin loader alive until every component object is destroyed.
  std::unique_ptr<class_loader::ClassLoader> loader;
  std::shared_ptr<rclcpp_components::NodeFactory> factory;
  std::unique_ptr<rclcpp_components::NodeInstanceWrapper> component;
  rclcpp::Node::SharedPtr harness;
  rclcpp::executors::SingleThreadedExecutor component_executor;
  rclcpp::executors::SingleThreadedExecutor harness_executor;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub;
  rclcpp::Publisher<PacketMsg>::SharedPtr lidar_pub;
  rclcpp::Publisher<PacketMsg>::SharedPtr imu_pub;
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
  std::vector<CloudMsg::ConstSharedPtr> clouds;
  std::vector<uint64_t> imu_timestamps;
  std::string ns;
  size_t width = 2048;
  size_t frames = 3;
  size_t imu_per_frame = 10;
  uint64_t period_ns = 100000000ULL;
  SensorInfo info = default_sensor_info(LidarMode(2048, 10));

  void SetUp() override {
    static size_t instance = 0;
    ns = "/raw_packet_qos_test_" + std::to_string(++instance);
    info.format.pixels_per_column = HEIGHT;
    info.format.pixel_shift_by_row.assign(HEIGHT, 0);
    info.beam_altitude_angles.assign(HEIGHT, 0.0);
    info.beam_azimuth_angles.assign(HEIGHT, 0.0);
    // Serialized metadata treats UDP port zero as a disabled stream.
    // These synthetic ports are descriptive only; the test uses ROS topics.
    info.config.udp_port_lidar = 7502;
    info.config.udp_port_imu = 7503;
    ASSERT_EQ(get_format(info).lidar_packet_size, 24896u);
    harness = std::make_shared<rclcpp::Node>("burst_harness", ns);
    harness_executor.add_node(harness);
    metadata_pub = harness->create_publisher<std_msgs::msg::String>(
        "metadata", rclcpp::QoS(1).reliable().transient_local());
    lidar_pub = harness->create_publisher<PacketMsg>(
        "lidar_packets", rclcpp::QoS(1024).reliable());
    imu_pub = harness->create_publisher<PacketMsg>(
        "imu_packets", rclcpp::QoS(1024).reliable());
    cloud_sub = harness->create_subscription<CloudMsg>(
        "points", rclcpp::QoS(10).best_effort(),
        [this](CloudMsg::ConstSharedPtr msg) {
          clouds.push_back(std::move(msg));
        });
    imu_sub = harness->create_subscription<ImuMsg>(
        "imu", rclcpp::QoS(64).best_effort(),
        [this](ImuMsg::ConstSharedPtr msg) {
          imu_timestamps.push_back(
              rclcpp::Time(msg->header.stamp).nanoseconds());
        });
  }

  void TearDown() override {
    if (component)
      component_executor.remove_node(component->get_node_base_interface());
    component.reset();
    factory.reset();
    loader.reset();
    harness_executor.remove_node(harness);
  }

  bool spin_until(const std::function<bool()> &condition,
                  std::chrono::seconds timeout = 10s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      component_executor.spin_some(1ms);
      harness_executor.spin_some(1ms);
      if (condition())
        return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  void start_component(bool system_default_outputs = false,
                       const std::vector<rclcpp::Parameter> &extra = {}) {
    // An explicit override permits a negative-control run against a prior
    // installed component without rebuilding or modifying either library.
    const char *library = std::getenv("OUSTER_TEST_COMPONENT_LIBRARY");
    loader = std::make_unique<class_loader::ClassLoader>(
        library ? library : OUSTER_CLOUD_COMPONENT_LIBRARY);
    const auto classes =
        loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    ASSERT_EQ(classes.size(), 1u);
    factory =
        loader->createInstance<rclcpp_components::NodeFactory>(classes.front());
    std::vector<rclcpp::Parameter> overrides{
        {"proc_mask", "IMU|PCL"},
        {"timestamp_mode", "TIME_FROM_INTERNAL_OSC"},
        {"point_type", "original"},
        {"organized", true},
        {"destagger", false},
        {"pub_static_tf", false},
        {"use_system_default_qos", system_default_outputs}};
    overrides.insert(overrides.end(), extra.begin(), extra.end());
    auto options = rclcpp::NodeOptions()
                       .use_intra_process_comms(false)
                       .arguments({"--ros-args", "-r", "__ns:=" + ns})
                       .parameter_overrides(overrides);
    component = std::make_unique<rclcpp_components::NodeInstanceWrapper>(
        factory->create_node_instance(options));
    component_executor.add_node(component->get_node_base_interface());
    std_msgs::msg::String metadata;
    metadata.data = info.to_json_string();
    metadata_pub->publish(metadata);
    ASSERT_TRUE(spin_until([this] {
      return lidar_pub->get_subscription_count() == 1 &&
             imu_pub->get_subscription_count() == 1 &&
             cloud_sub->get_publisher_count() == 1 &&
             imu_sub->get_publisher_count() == 1;
    }));
  }

  void expect_raw_endpoints(size_t depth,
                            rmw_qos_reliability_policy_t reliability) {
    for (const auto *topic : {"lidar_packets", "imu_packets"}) {
      const auto name = ns + "/" + topic;
      const auto endpoints = harness->get_subscriptions_info_by_topic(name);
      ASSERT_EQ(endpoints.size(), 1u);
      EXPECT_EQ(
          endpoints.front().qos_profile().get_rmw_qos_profile().reliability,
          reliability);
      // DDS discovery does not carry history/depth on every middleware.
      // Ask the loaded component's real subscription for its actual QoS.
      rclcpp::SubscriptionBase::SharedPtr subscription;
      component->get_node_base_interface()->for_each_callback_group(
          [&](rclcpp::CallbackGroup::SharedPtr group) {
            const auto found = group->find_subscription_ptrs_if(
                [&name](const auto &candidate) {
                  return name == candidate->get_topic_name();
                });
            if (found)
              subscription = found;
          });
      ASSERT_NE(subscription, nullptr);
      const auto qos = subscription->get_actual_qos().get_rmw_qos_profile();
      EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
      EXPECT_EQ(qos.depth, depth);
      EXPECT_EQ(qos.reliability, reliability);
      EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    }
  }

  void publish_frame(size_t frame) {
    const auto &format = get_format(info);
    impl::PacketWriter writer(format);
    std::vector<uint32_t> ranges(HEIGHT * width);
    for (size_t ring = 0; ring < HEIGHT; ++ring)
      for (size_t column = 0; column < width; ++column)
        ranges[ring * width + column] = expected_range(frame, ring, column);
    for (size_t first = 0; first < width; first += format.columns_per_packet) {
      PacketMsg packet;
      packet.buf.resize(format.lidar_packet_size);
      writer.set_frame_id(packet.buf.data(), frame);
      for (int i = 0; i < format.columns_per_packet; ++i) {
        auto *column = writer.nth_col(i, packet.buf.data());
        writer.set_col_measurement_id(column, first + i);
        writer.set_col_status(column, 1);
        writer.set_col_timestamp(column, START_NS + frame * period_ns +
                                             (first + i) * period_ns / width);
      }
      writer.set_block(ranges.data(), width, ChanField::RANGE,
                       packet.buf.data());
      lidar_pub->publish(packet);
    }
    for (size_t i = 0; i < imu_per_frame; ++i) {
      PacketMsg packet;
      packet.buf.resize(format.imu_packet_size);
      const uint64_t stamp =
          START_NS + frame * period_ns + i * IMU_PERIOD_NS + 42;
      for (size_t field = 0; field < 3; ++field)
        for (size_t byte = 0; byte < 8; ++byte)
          packet.buf[field * 8 + byte] =
              static_cast<uint8_t>(stamp >> (8 * byte));
      imu_pub->publish(packet);
    }
  }

  void expect_clouds_and_imu() {
    ASSERT_EQ(clouds.size(), frames);
    for (size_t frame = 0; frame < frames; ++frame) {
      const auto &cloud = *clouds[frame];
      SCOPED_TRACE("frame " + std::to_string(frame));
      ASSERT_EQ(rclcpp::Time(cloud.header.stamp).nanoseconds(),
                START_NS + frame * period_ns);
      ASSERT_EQ(cloud.width, width);
      ASSERT_EQ(cloud.height, HEIGHT);
      ASSERT_FALSE(cloud.is_bigendian);
      const auto find_field = [&cloud](const std::string &name) {
        return std::find_if(
            cloud.fields.begin(), cloud.fields.end(),
            [&name](const auto &field) { return field.name == name; });
      };
      const auto time = find_field("t");
      const auto range = find_field("range");
      ASSERT_NE(time, cloud.fields.end());
      ASSERT_NE(range, cloud.fields.end());
      ASSERT_EQ(time->datatype, sensor_msgs::msg::PointField::UINT32);
      ASSERT_EQ(range->datatype, sensor_msgs::msg::PointField::UINT32);
      ASSERT_GE(cloud.data.size(), cloud.row_step * HEIGHT);
      for (size_t ring = 0; ring < HEIGHT; ++ring) {
        for (size_t column = 0; column < width; ++column) {
          const auto *point = cloud.data.data() + ring * cloud.row_step +
                              column * cloud.point_step;
          uint32_t observed_time, observed_range;
          std::memcpy(&observed_time, point + time->offset,
                      sizeof(observed_time));
          std::memcpy(&observed_range, point + range->offset,
                      sizeof(observed_range));
          ASSERT_EQ(observed_time, column * period_ns / width)
              << "ring=" << ring << " column=" << column;
          ASSERT_EQ(observed_range, expected_range(frame, ring, column))
              << "ring=" << ring << " column=" << column;
        }
      }
    }
    ASSERT_EQ(imu_timestamps.size(), frames * imu_per_frame);
    for (size_t i = 0; i < imu_timestamps.size(); ++i)
      EXPECT_EQ(imu_timestamps[i], START_NS + i * IMU_PERIOD_NS + 42);
  }
};

class RawPacketBurstTest : public RawPacketQosComponentTest,
                           public ::testing::WithParamInterface<int> {
protected:
  void SetUp() override {
    const int frequency = GetParam();
    width = frequency == 10 ? 2048 : 1024;
    frames = frequency == 10 ? 3 : 6;
    imu_per_frame = frequency == 10 ? 10 : 5;
    period_ns = 1000000000ULL / frequency;
    info = default_sensor_info(LidarMode(width, frequency));
    RawPacketQosComponentTest::SetUp();
  }
};

TEST_P(RawPacketBurstTest, RawQueuesRetainCompleteFramesWhileExecutorIsPaused) {
  start_component();
  ASSERT_FALSE(HasFatalFailure());
  expect_raw_endpoints(1024, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  // Deliberately do not spin the component executor while publishing. DDS
  // acknowledgements prove reception before callbacks can consume any packet.
  for (size_t frame = 0; frame < frames; ++frame)
    publish_frame(frame);
  ASSERT_TRUE(lidar_pub->wait_for_all_acked(5s));
  ASSERT_TRUE(imu_pub->wait_for_all_acked(5s));
  EXPECT_TRUE(clouds.empty());
  EXPECT_TRUE(imu_timestamps.empty());
  ASSERT_TRUE(spin_until([this] {
    return clouds.size() >= frames &&
           imu_timestamps.size() >= frames * imu_per_frame;
  }));
  expect_clouds_and_imu();
}

INSTANTIATE_TEST_SUITE_P(EqualBandwidthModes, RawPacketBurstTest,
                         ::testing::Values(10, 20), [](const auto &mode) {
                           return mode.param == 10 ? "Legacy2048x10"
                                                   : "Legacy1024x20";
                         });

TEST_F(RawPacketQosComponentTest,
       SystemDefaultOutputQosDoesNotChangeRawInputQos) {
  start_component(true);
  ASSERT_FALSE(HasFatalFailure());
  expect_raw_endpoints(1024, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
}

TEST_F(RawPacketQosComponentTest, ExplicitRawProfileReachesBothSubscriptions) {
  start_component(false, {{"raw_packet_qos_depth", 32},
                          {"raw_packet_qos_reliability", "best_effort"}});
  ASSERT_FALSE(HasFatalFailure());
  expect_raw_endpoints(32, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

} // namespace
