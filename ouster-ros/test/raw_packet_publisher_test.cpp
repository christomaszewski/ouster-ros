#include <gtest/gtest.h>
#include <ouster/client.h>
#include <ouster/impl/netcompat.h>
#include <ouster/packet.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <unistd.h>
#endif

#include "ouster_ros/raw_packet_publisher.h"

namespace {

using namespace std::chrono_literals;
using ouster_ros::RawPacketPublisher;
using PacketMsg = RawPacketPublisher::PacketMsg;
namespace sensor = ouster::sdk::sensor;
namespace core = ouster::sdk::core;

std::vector<uint8_t> packet(size_t size, uint8_t sequence) {
    std::vector<uint8_t> bytes(size);
    for (size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = static_cast<uint8_t>(sequence * 37 + i);
    }
    return bytes;
}

template <typename Predicate>
bool wait_until(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}

// Block exactly the first publish, including access to its message bytes.
// This detects both lock coupling and ownership of the in-flight buffer.
class PublishGate {
   public:
    void publish(const PacketMsg& message) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!entered_) {
            entered_ = true;
            changed_.notify_all();
            if (!changed_.wait_for(lock, 5s, [this] { return released_; })) {
                timed_out_ = true;
            }
        }
        messages_.push_back(message.buf);
    }

    bool wait_for_entry() {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, 2s, [this] { return entered_; });
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }

    std::vector<std::vector<uint8_t>> messages() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_;
    }

    bool timed_out() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return timed_out_;
    }

   private:
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    bool entered_ = false;
    bool released_ = false;
    bool timed_out_ = false;
    std::vector<std::vector<uint8_t>> messages_;
};

struct ReleaseGateOnExit {
    PublishGate& gate;
    ~ReleaseGateOnExit() { gate.release(); }
};

TEST(RawPacketPublisherTest,
     StalledPublisherCannotBlockProducerOrAliasBuffers) {
    constexpr size_t size = 24896;
    PublishGate gate;
    RawPacketPublisher publisher(
        2, size, [&](const auto& message) { gate.publish(message); });
    ReleaseGateOnExit release{gate};
    auto bytes = packet(size, 0);
    ASSERT_TRUE(publisher.enqueue(bytes));
    ASSERT_TRUE(gate.wait_for_entry());

    auto producer = std::async(std::launch::async, [&] {
        std::vector<bool> accepted;
        for (uint8_t sequence = 1; sequence <= 3; ++sequence) {
            bytes = packet(size, sequence);
            accepted.push_back(publisher.enqueue(bytes));
        }
        bytes.assign(size,
                     0xff);  // Neither pending nor in-flight may alias this.
        return accepted;
    });
    const bool progressed = producer.wait_for(1s) == std::future_status::ready;
    if (!progressed)
        gate.release();  // Always unblock before future destruction.
    ASSERT_TRUE(progressed);
    EXPECT_EQ(producer.get(), (std::vector<bool>{true, true, false}));
    const auto blocked = publisher.stats();
    EXPECT_EQ(blocked.accepted, 3u);
    EXPECT_EQ(blocked.overflow, 1u);
    EXPECT_EQ(blocked.queued, 2u);
    EXPECT_TRUE(blocked.in_flight);
    EXPECT_EQ(blocked.published, 0u);
    gate.release();
    ASSERT_TRUE(wait_until([&] { return publisher.stats().published == 3; }));
    publisher.stop();
    EXPECT_EQ(gate.messages(),
              (std::vector<std::vector<uint8_t>>{
                  packet(size, 0), packet(size, 1), packet(size, 2)}));
    EXPECT_FALSE(gate.timed_out());
    EXPECT_EQ(publisher.stats().queued, 0u);
    EXPECT_FALSE(publisher.stats().in_flight);
}

class LoopbackReceiver {
   public:
    LoopbackReceiver() {
        client = sensor::init_client("127.0.0.1", 0, 0);
        if (!client)
            throw std::runtime_error("Failed to create loopback client");
        sender = socket(AF_INET, SOCK_DGRAM, 0);
        if (sender == SOCKET_ERROR)
            throw std::runtime_error("Failed to create UDP sender");
        destination.sin_family = AF_INET;
        destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        destination.sin_port = htons(sensor::get_lidar_port(*client));
    }

    ~LoopbackReceiver() {
        if (sender != SOCKET_ERROR) {
#ifdef _WIN32
            closesocket(sender);
#else
            close(sender);
#endif
        }
    }

    bool send(const std::vector<uint8_t>& bytes) {
        const auto sent =
            sendto(sender, reinterpret_cast<const char*>(bytes.data()),
                   static_cast<int>(bytes.size()), 0,
                   reinterpret_cast<const sockaddr*>(&destination),
                   sizeof(destination));
        return sent == static_cast<decltype(sent)>(bytes.size());
    }

    bool receive(core::LidarPacket& packet) {
        return (sensor::poll_client(*client, 1) & sensor::LIDAR_DATA) &&
               sensor::read_lidar_packet(*client, packet);
    }

    std::shared_ptr<sensor::Client> client;

   private:
    SOCKET sender{static_cast<SOCKET>(SOCKET_ERROR)};
    sockaddr_in destination{};
};

TEST(RawPacketPublisherTest, UdpReceptionContinuesWhilePublishingIsBlocked) {
    constexpr size_t size = 24896;
    LoopbackReceiver udp;
    PublishGate gate;
    RawPacketPublisher publisher(
        4, size, [&](const auto& message) { gate.publish(message); });
    ReleaseGateOnExit release{gate};
    core::LidarPacket input(size);
    ASSERT_TRUE(udp.send(packet(size, 0)));
    ASSERT_TRUE(udp.receive(input));
    ASSERT_TRUE(publisher.enqueue(input.buf));
    ASSERT_TRUE(gate.wait_for_entry());

    auto receiver = std::async(std::launch::async, [&] {
        size_t received = 0;
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (received < 8 && std::chrono::steady_clock::now() < deadline) {
            if (udp.receive(input)) {
                ++received;
                publisher.enqueue(input.buf);
            }
        }
        return received;
    });
    ReleaseGateOnExit release_before_receiver_destructs{gate};
    for (uint8_t sequence = 1; sequence <= 8; ++sequence) {
        ASSERT_TRUE(udp.send(packet(size, sequence)));
    }
    const bool progressed = receiver.wait_for(1s) == std::future_status::ready;
    if (!progressed) gate.release();
    ASSERT_TRUE(progressed);
    EXPECT_EQ(receiver.get(), 8u);
    const auto blocked = publisher.stats();
    EXPECT_EQ(blocked.accepted, 5u);
    EXPECT_EQ(blocked.overflow, 4u);
    EXPECT_EQ(blocked.queued, 4u);
    EXPECT_TRUE(blocked.in_flight);
    EXPECT_EQ(blocked.published, 0u);
    gate.release();
    ASSERT_TRUE(wait_until([&] { return publisher.stats().published == 5; }));
    publisher.stop();
    const auto messages = gate.messages();
    ASSERT_EQ(messages.size(), 5u);
    for (uint8_t i = 0; i < messages.size(); ++i)
        EXPECT_EQ(messages[i], packet(size, i));
    EXPECT_FALSE(gate.timed_out());
}

TEST(RawPacketPublisherTest, LidarBackpressureDoesNotBlockImuPublication) {
    PublishGate lidar_gate;
    RawPacketPublisher lidar(
        2, 24896, [&](const auto& message) { lidar_gate.publish(message); });
    ReleaseGateOnExit release{lidar_gate};
    std::mutex imu_mutex;
    std::vector<std::vector<uint8_t>> imu_messages;
    RawPacketPublisher imu(4, 48, [&](const auto& message) {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_messages.push_back(message.buf);
    });
    ASSERT_TRUE(lidar.enqueue(packet(24896, 0)));
    ASSERT_TRUE(lidar_gate.wait_for_entry());
    for (uint8_t i = 0; i < 4; ++i) ASSERT_TRUE(imu.enqueue(packet(48, i)));
    ASSERT_TRUE(wait_until([&] { return imu.stats().published == 4; }));
    EXPECT_EQ(lidar.stats().published, 0u);
    EXPECT_TRUE(lidar.stats().in_flight);
    imu.stop();
    std::lock_guard<std::mutex> lock(imu_mutex);
    ASSERT_EQ(imu_messages.size(), 4u);
    for (uint8_t i = 0; i < 4; ++i) EXPECT_EQ(imu_messages[i], packet(48, i));
}

TEST(RawPacketPublisherTest, PublishExceptionsAreCountedAndWorkerContinues) {
    size_t attempts = 0;
    std::vector<std::vector<uint8_t>> messages;
    RawPacketPublisher publisher(4, 48, [&](const auto& message) {
        if (++attempts == 1)
            throw std::runtime_error("synthetic publish failure");
        messages.push_back(message.buf);
    });
    for (uint8_t i = 0; i < 3; ++i)
        ASSERT_TRUE(publisher.enqueue(packet(48, i)));
    ASSERT_TRUE(wait_until([&] { return publisher.stats().published == 2; }));
    publisher.stop();
    const auto stats = publisher.stats();
    EXPECT_EQ(stats.accepted, 3u);
    EXPECT_EQ(stats.publish_errors, 1u);
    EXPECT_EQ(stats.published, 2u);
    EXPECT_EQ(stats.overflow, 0u);
    EXPECT_EQ(messages, (std::vector<std::vector<uint8_t>>{packet(48, 1),
                                                           packet(48, 2)}));
}

TEST(RawPacketPublisherTest,
     StopDiscardsPendingPacketsAndFreshWorkerHasNoStaleData) {
    PublishGate gate;
    RawPacketPublisher publisher(
        2, 48, [&](const auto& message) { gate.publish(message); });
    ReleaseGateOnExit release{gate};
    ASSERT_TRUE(publisher.enqueue(packet(48, 0)));
    ASSERT_TRUE(gate.wait_for_entry());
    ASSERT_TRUE(publisher.enqueue(packet(48, 1)));
    ASSERT_TRUE(publisher.enqueue(packet(48, 2)));
    publisher.request_stop();
    publisher
        .request_stop();  // Closing twice cannot double-count discarded data.
    EXPECT_FALSE(publisher.enqueue(packet(48, 3)));
    const auto closing = publisher.stats();
    EXPECT_EQ(closing.discarded_on_stop, 2u);
    EXPECT_EQ(closing.queued, 0u);
    EXPECT_TRUE(closing.in_flight);
    EXPECT_EQ(closing.rejected_stopped, 1u);

    auto stopped = std::async(std::launch::async, [&] { publisher.stop(); });
    ReleaseGateOnExit release_before_future_destructs{gate};
    EXPECT_EQ(stopped.wait_for(20ms), std::future_status::timeout);
    gate.release();
    ASSERT_EQ(stopped.wait_for(2s), std::future_status::ready);
    stopped.get();
    publisher.stop();
    EXPECT_EQ(gate.messages(),
              (std::vector<std::vector<uint8_t>>{packet(48, 0)}));
    EXPECT_EQ(publisher.stats().published, 1u);
    EXPECT_FALSE(publisher.stats().in_flight);
    EXPECT_FALSE(gate.timed_out());

    // Reactivation creates fresh storage, potentially for a changed profile.
    std::vector<std::vector<uint8_t>> restarted_messages;
    RawPacketPublisher restarted(2, 12608, [&](const auto& message) {
        restarted_messages.push_back(message.buf);
    });
    ASSERT_TRUE(restarted.enqueue(packet(12608, 9)));
    ASSERT_TRUE(wait_until([&] { return restarted.stats().published == 1; }));
    restarted.stop();
    EXPECT_EQ(restarted_messages,
              (std::vector<std::vector<uint8_t>>{packet(12608, 9)}));
    EXPECT_EQ(restarted.stats().accepted, 1u);
    EXPECT_EQ(restarted.stats().discarded_on_stop, 0u);
}

TEST(RawPacketPublisherTest,
     InvalidPacketSizesAreRejectedWithoutConsumingQueueSpace) {
    RawPacketPublisher publisher(2, 48, [](const auto&) {});
    for (const size_t size : {size_t{0}, size_t{47}, size_t{49}}) {
        EXPECT_FALSE(publisher.enqueue(packet(size, 0)));
    }
    EXPECT_EQ(publisher.stats().invalid_size, 3u);
    EXPECT_EQ(publisher.stats().accepted, 0u);
    ASSERT_TRUE(publisher.enqueue(packet(48, 1)));
    ASSERT_TRUE(wait_until([&] { return publisher.stats().published == 1; }));
    publisher.stop();
    EXPECT_EQ(publisher.stats().overflow, 0u);
}

TEST(RawPacketPublisherTest,
     InvalidCapacitySizeAndCallbackFailBeforeWorkerStarts) {
    for (const size_t depth : {size_t{0}, size_t{65537}}) {
        EXPECT_THROW(RawPacketPublisher(depth, 48, [](const auto&) {}),
                     std::invalid_argument);
    }
    EXPECT_THROW(RawPacketPublisher(1, 0, [](const auto&) {}),
                 std::invalid_argument);
    EXPECT_THROW(RawPacketPublisher(1, 48, {}), std::invalid_argument);
}

}  // namespace
