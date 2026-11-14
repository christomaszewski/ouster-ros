#pragma once

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <ouster_sensor_msgs/msg/telemetry.hpp>

namespace ouster_ros {

// Optional heartbeat rate for unchanged status. Transitions and countdown
// changes remain immediate; packet-derived headers are never synthesized.
class TelemetryLimiter {
   public:
    using Clock = std::chrono::steady_clock;
    using Message = ouster_sensor_msgs::msg::Telemetry;

    explicit TelemetryLimiter(double rate_hz = 0.0) {
        if (!std::isfinite(rate_hz) || rate_hz < 0.0)
            throw std::invalid_argument("telemetry_max_rate_hz must be finite and nonnegative");
        period_seconds_ = rate_hz == 0.0 ? 0.0 : 1.0 / rate_hz;
        if (!std::isfinite(period_seconds_))
            throw std::invalid_argument("telemetry_max_rate_hz is too small");
    }

    bool should_publish(const Message& message) {
        // Preserve the default packet cadence without a per-packet clock read.
        return period_seconds_ == 0.0 || should_publish(message, Clock::now());
    }

    bool should_publish(const Message& message, Clock::time_point now) {
        if (period_seconds_ == 0.0) return true;
        const auto status = std::make_tuple(message.thermal_shutdown,
            message.shot_limiting, message.countdown_thermal_shutdown,
            message.countdown_shot_limiting);
        if (!initialized_ || status != status_ || now < last_ ||
            std::chrono::duration<double>(now - last_).count() >= period_seconds_) {
            initialized_ = true;
            status_ = status;
            last_ = now;
            return true;
        }
        return false;
    }

   private:
    double period_seconds_ = 0.0;
    bool initialized_ = false;
    Clock::time_point last_{};
    std::tuple<uint8_t, uint8_t, uint16_t, uint16_t> status_{};
};

}  // namespace ouster_ros
