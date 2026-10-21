#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "ouster_sensor_msgs/msg/packet_msg.hpp"

namespace ouster_ros {

// One receive producer and one publishing worker per stream. The short queue
// lock is NEVER held across publish(): middleware backpressure cannot reach the
// UDP receive thread. All packet storage is allocated before reception starts.
class RawPacketPublisher {
public:
  using PacketMsg = ouster_sensor_msgs::msg::PacketMsg;
  using PublishFunction = std::function<void(const PacketMsg &)>;

  struct Stats {
    uint64_t accepted = 0;
    uint64_t published = 0;
    uint64_t overflow = 0;
    uint64_t publish_errors = 0;
    uint64_t discarded_on_stop = 0;
    uint64_t invalid_size = 0;
    uint64_t rejected_stopped = 0;
    size_t queued = 0;
    bool in_flight = false;
  };

  RawPacketPublisher(size_t depth, size_t packet_size, PublishFunction publish)
      : packet_size_(packet_size), publish_(std::move(publish)) {
    if (depth < 1 || depth > 65536 || packet_size == 0 || !publish_) {
      throw std::invalid_argument("Invalid raw packet publishing queue");
    }
    slots_.resize(depth);
    for (auto &slot : slots_)
      slot.buf.resize(packet_size_);
    outgoing_.buf.resize(packet_size_);
    worker_ = std::thread([this] { run(); });
  }

  ~RawPacketPublisher() { stop(); }
  RawPacketPublisher(const RawPacketPublisher &) = delete;
  RawPacketPublisher &operator=(const RawPacketPublisher &) = delete;

  bool enqueue(const std::vector<uint8_t> &bytes) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        ++stats_.rejected_stopped;
        return false;
      }
      if (bytes.size() != packet_size_) {
        ++stats_.invalid_size;
        return false;
      }
      if (stats_.queued == slots_.size()) {
        ++stats_.overflow;
        return false; // Drop newest; preserve accepted packet order.
      }
      std::memcpy(slots_[tail_].buf.data(), bytes.data(), packet_size_);
      tail_ = (tail_ + 1) % slots_.size();
      ++stats_.queued;
      ++stats_.accepted;
    }
    ready_.notify_one();
    return true;
  }

  // Lifecycle operations are serialized by the owner, after stopping the
  // receive producer. Close both streams before joining either worker.
  void request_stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      stats_.discarded_on_stop += stats_.queued;
      stats_.queued = 0;
    }
    ready_.notify_one();
  }

  void stop() {
    request_stop();
    // rclcpp has no portable cancellation of an in-flight publish. Joining
    // keeps both message and publisher alive until middleware returns.
    if (worker_.joinable())
      worker_.join();
  }

  Stats stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

private:
  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      ready_.wait(lock, [this] { return stopping_ || stats_.queued > 0; });
      if (stopping_)
        return;
      outgoing_.buf.swap(slots_[head_].buf);
      head_ = (head_ + 1) % slots_.size();
      --stats_.queued;
      stats_.in_flight = true;
      lock.unlock();
      bool published = false;
      try {
        publish_(outgoing_);
        published = true;
      } catch (...) {
        // A middleware error must not terminate the process or UDP
        // reception. The owner reports these counters off that path.
      }
      lock.lock();
      if (published)
        ++stats_.published;
      else
        ++stats_.publish_errors;
      stats_.in_flight = false;
    }
  }

  const size_t packet_size_;
  const PublishFunction publish_;
  std::vector<PacketMsg> slots_;
  PacketMsg outgoing_;
  size_t head_ = 0;
  size_t tail_ = 0;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_ = false;
  Stats stats_;
  std::thread worker_;
};

} // namespace ouster_ros
