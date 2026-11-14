#include <gtest/gtest.h>
#include <limits>
#include "src/telemetry_limiter.h"

namespace ouster_ros {
namespace {
using namespace std::chrono_literals;
using Clock = TelemetryLimiter::Clock;
using Message = TelemetryLimiter::Message;

TEST(TelemetryLimiter, DefaultPreservesEveryPacket) {
    TelemetryLimiter gate;
    Message message;
    for (int i = 0; i < 4096; ++i)
        EXPECT_TRUE(gate.should_publish(message, Clock::time_point{}));
}

TEST(TelemetryLimiter, HeartbeatUsesSteadyTimeAndPreservesHeader) {
    TelemetryLimiter gate(10.0);
    Message message;
    auto now = Clock::time_point{};
    EXPECT_TRUE(gate.should_publish(message, now));
    for (int i = 1; i < 100; ++i) {
        message.header.stamp.sec = i;
        EXPECT_FALSE(gate.should_publish(message, now + i * 1ms));
    }
    EXPECT_TRUE(gate.should_publish(message, now + 100ms));
    EXPECT_EQ(message.header.stamp.sec, 99);
    EXPECT_FALSE(gate.should_publish(message, now + 199ms));
    EXPECT_TRUE(gate.should_publish(message, now + 200ms));
}

TEST(TelemetryLimiter, EveryStatusAndCountdownTransitionIsImmediate) {
    TelemetryLimiter gate(1.0);
    Message message;
    auto now = Clock::time_point{};
    ASSERT_TRUE(gate.should_publish(message, now));
    message.thermal_shutdown = 1;
    EXPECT_TRUE(gate.should_publish(message, now += 1ms));
    message.shot_limiting = 2;
    EXPECT_TRUE(gate.should_publish(message, now += 1ms));
    message.countdown_thermal_shutdown = 15;
    EXPECT_TRUE(gate.should_publish(message, now += 1ms));
    message.countdown_shot_limiting = 12;
    EXPECT_TRUE(gate.should_publish(message, now += 1ms));
    EXPECT_FALSE(gate.should_publish(message, now + 999ms));
    EXPECT_TRUE(gate.should_publish(message, now + 1000ms));
    message = Message{};
    EXPECT_TRUE(gate.should_publish(message, now + 1001ms));
}

TEST(TelemetryLimiter, NewHandlerAndClockRegressionRestartHeartbeat) {
    TelemetryLimiter gate(10.0);
    Message message;
    const auto now = Clock::time_point{} + 10s;
    EXPECT_TRUE(gate.should_publish(message, now));
    EXPECT_TRUE(gate.should_publish(message, now - 1s));
    EXPECT_FALSE(gate.should_publish(message, now - 1s + 1ms));
    gate = TelemetryLimiter(10.0);
    EXPECT_TRUE(gate.should_publish(message, now));
}

TEST(TelemetryLimiter, RejectsInvalidRates) {
    for (double rate : {-1.0, std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::denorm_min()})
        EXPECT_THROW(TelemetryLimiter{rate}, std::invalid_argument);
    EXPECT_NO_THROW(TelemetryLimiter{0.0});
    EXPECT_NO_THROW(TelemetryLimiter{0.01});
    EXPECT_NO_THROW(TelemetryLimiter{10000.0});
}
}  // namespace
}  // namespace ouster_ros
