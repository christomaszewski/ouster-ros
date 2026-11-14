// Include the concrete combined node so this test exercises its actual
// publisher lifecycle without connecting to a sensor or exporting a test API.
#include "../src/os_driver_node.cpp"
#include <dlfcn.h>
#include <gtest/gtest.h>
#include <rcl/rcl.h>
#include <atomic>
#include <cstring>
#include <limits>

namespace {
std::atomic<size_t> telemetry_publications{0};
using namespace ouster::sdk::core;

class DriverHarness : public ouster_ros::OusterDriver {
   public:
    DriverHarness() : OusterDriver(rclcpp::NodeOptions().use_intra_process_comms(false)) {}
    void open(const std::string& mask, double rate) {
        ASSERT_TRUE(set_parameter(rclcpp::Parameter("proc_mask", mask)).successful);
        ASSERT_TRUE(set_parameter(rclcpp::Parameter("telemetry_max_rate_hz", rate)).successful);
        info = default_sensor_info(LidarMode::_512x10);
        create_publishers();
    }
    void packet() {
        LidarPacket packet;
        packet.buf.assign(get_format(info).lidar_packet_size, 0);
        on_lidar_packet_msg(packet);
    }
    void close() { cleanup(); }
};

class TelemetryReconfigureTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override { telemetry_publications = 0; }
};

TEST_F(TelemetryReconfigureTest, RemovingTelemetryStopsActualPublicationAndReopenResetsHeartbeat) {
    DriverHarness node;
    node.open("TLM", 0.0);
    node.packet(); node.packet();
    ASSERT_EQ(telemetry_publications.load(), 2u);
    node.close();
    node.open("", 0.0);
    node.packet(); node.packet();
    EXPECT_EQ(telemetry_publications.load(), 2u);
    node.close();

    // A long heartbeat makes this deterministic without wall-clock sleeps.
    node.open("TLM", 0.000001);
    node.packet(); node.packet();
    EXPECT_EQ(telemetry_publications.load(), 3u);
    // Parameter updates take effect at publisher configuration, not midstream.
    ASSERT_TRUE(node.set_parameter(rclcpp::Parameter("telemetry_max_rate_hz", 0.0)).successful);
    node.packet();
    EXPECT_EQ(telemetry_publications.load(), 3u);
    node.close();
    node.open("TLM", 0.000001);
    node.packet(); node.packet();
    EXPECT_EQ(telemetry_publications.load(), 4u);
    node.close();
    node.open("TLM", 0.0);
    node.packet(); node.packet();
    EXPECT_EQ(telemetry_publications.load(), 6u);
    node.close(); node.close();
}

TEST_F(TelemetryReconfigureTest, InvalidConfiguredRateDoesNotLeaveOldTelemetryActive) {
    DriverHarness node;
    node.open("TLM", 0.0);
    node.packet();
    ASSERT_EQ(telemetry_publications.load(), 1u);
    node.close();
    for (double rate : {-1.0, std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(node.open("TLM", rate), std::invalid_argument);
        node.packet();
        EXPECT_EQ(telemetry_publications.load(), 1u);
        node.close();
    }
}
}  // namespace

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* publisher,
                                 const void* message,
                                 rmw_publisher_allocation_t* allocation) {
    using Publish = rcl_ret_t (*)(const rcl_publisher_t*, const void*, rmw_publisher_allocation_t*);
    static const auto actual = reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rcl_publish"));
    if (!actual) return RCL_RET_ERROR;
    const char* topic = rcl_publisher_get_topic_name(publisher);
    if (topic && std::strcmp(topic, "/telemetry") == 0) ++telemetry_publications;
    return actual(publisher, message, allocation);
}
