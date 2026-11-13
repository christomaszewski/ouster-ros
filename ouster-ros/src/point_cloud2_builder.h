#pragma once

#include <cstring>
#include <pcl/for_each_type.h>
#include <pcl/point_struct_traits.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ouster_ros {

namespace detail {
template <typename PointT>
struct CloudFieldAdder {
    sensor_msgs::msg::PointCloud2& message;

    template <typename Field>
    void operator()() const {
        sensor_msgs::msg::PointField field;
        field.name = pcl::traits::name<PointT, Field>::value;
        field.offset = pcl::traits::offset<PointT, Field>::value;
        field.datatype = pcl::traits::datatype<PointT, Field>::value;
        field.count = pcl::traits::datatype<PointT, Field>::size;
        message.fields.push_back(std::move(field));
    }
};

template <typename PointT>
struct CloudFieldWriter {
    uint8_t* destination;
    const PointT& point;

    template <typename Field>
    void operator()() const {
        constexpr auto offset = pcl::traits::offset<PointT, Field>::value;
        constexpr auto size =
            sizeof(typename pcl::traits::datatype<PointT, Field>::type);
        static_assert(offset + size <= sizeof(PointT));
        // Fixed offsets/sizes allow the compiler to emit ordinary stores.
        // The destination remains byte storage: no alignment or object-lifetime
        // assumptions about PointCloud2::data are made.
        std::memcpy(destination + offset,
                    reinterpret_cast<const uint8_t*>(&point) + offset, size);
    }
};
}  // namespace detail

// A single correctly aligned point is converted at a time, then its registered
// fields are written into the final message. There is no full-size PCL cloud or
// second whole-cloud copy. Padding retains its original offsets/stride and is
// zero-initialized by the byte vector instead of exposing object padding. The
// initialized fourth float in PCL's XYZ block is preserved as well: some PCL
// consumers copy the gaps between registered fields into their point objects.
template <typename PointT>
class PointCloud2Builder {
   public:
    using PointType = PointT;

    PointCloud2Builder(uint32_t width, uint32_t height)
        : width_(width), height_(height) {}

    void attach(sensor_msgs::msg::PointCloud2& message) {
        message_ = &message;
        if (message.fields.empty()) {
            pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(
                detail::CloudFieldAdder<PointT>{message});
            message.point_step = sizeof(PointT);
            // Matches the existing PCL conversion on the supported platforms.
            message.is_bigendian = false;
        }
    }

    void prepare(bool organized) {
        organized_ = organized;
        count_ = organized ? static_cast<size_t>(width_) * height_ : 0;
        const auto maximum_bytes =
            static_cast<size_t>(width_) * height_ * sizeof(PointT);
        message_->data.reserve(maximum_bytes);
        if (organized) message_->data.resize(maximum_bytes);
        data_ = message_->data.data();
        available_bytes_ = message_->data.size();
        // Unorganized output retains its previous size until finish(). Grow
        // only the newly required tail, rather than zeroing a full frame when
        // the number of valid points fluctuates.
    }

    size_t size() const { return count_; }

    PointT& point(size_t, bool organized, PointT& converted_point) {
        if (!organized) ++count_;
        return converted_point;
    }

    void store(size_t index, const PointT& point, bool organized) {
        const size_t end = (index + 1) * sizeof(PointT);
        if (!organized && end > available_bytes_) {
            message_->data.resize(end);
            available_bytes_ = end;
        }
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(
            detail::CloudFieldWriter<PointT>{data_ + index * sizeof(PointT), point});
        static_assert(pcl::traits::offset<PointT, pcl::fields::x>::value == 0 &&
                      pcl::traits::offset<PointT, pcl::fields::y>::value == 4 &&
                      pcl::traits::offset<PointT, pcl::fields::z>::value == 8 &&
                      sizeof(PointT) >= 16);
        // All supported types initialize this slot: homogeneous 1.0f for the
        // PCL XYZ block, or the registered intensity in compact PointXYZI.
        // Copy its representation without reading an inactive union member.
        std::memcpy(data_ + index * sizeof(PointT) + 12,
                    reinterpret_cast<const uint8_t*>(&point) + 12, 4);
    }

    void finish(bool dense) {
        message_->width = organized_ ? width_ : static_cast<uint32_t>(count_);
        message_->height = organized_ ? height_ : 1;
        message_->row_step = message_->width * sizeof(PointT);
        message_->is_dense = dense;
        message_->data.resize(count_ * sizeof(PointT));
    }

   private:
    uint32_t width_;
    uint32_t height_;
    bool organized_ = true;
    size_t count_ = 0;
    size_t available_bytes_ = 0;
    uint8_t* data_ = nullptr;
    sensor_msgs::msg::PointCloud2* message_ = nullptr;
};

}  // namespace ouster_ros
