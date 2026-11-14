// Frozen 3bf8744 image processor for independent optimization equivalence.
/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_cloud_producer.h
 * @brief takes in a lidar scan object and produces a PointCloud2 message
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ouster/image_processing.h"
#include "../src/lidar_packet_handler.h"

namespace ouster_ros {

namespace ChanField = ouster::sdk::core::ChanField;

class FrozenImageProcessor {
   public:
    using OutputType =
        std::map<std::string, std::shared_ptr<sensor_msgs::msg::Image>>;
    using PostProcessingFn = std::function<void(OutputType)>;
    using DemandFn = std::function<bool(const std::string&)>;

    // ROS discovery is asynchronous. Snapshot this predicate once per scan;
    // a new subscriber may miss the scan that precedes discovery. The native
    // scan worker alone owns image/exposure state.
    template <typename Publishers>
    static DemandFn subscriber_demand(Publishers publishers, bool always_process) {
        return [publishers = std::move(publishers), always_process](
                   const std::string& channel) {
            if (always_process) return true;
            const auto it = publishers.find(channel);
            return it != publishers.end() &&
                   (it->second->get_subscription_count() != 0 ||
                    it->second->get_intra_process_subscription_count() != 0);
        };
    }

   public:
    FrozenImageProcessor(const ouster::sdk::core::SensorInfo& info,
                   const std::string& frame_id,
                   const std::string& mask_path,
                   PostProcessingFn func, DemandFn demand = {})
        : frame(frame_id), post_processing_fn(func), demand_fn(std::move(demand)),
          info_(info) {
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        if (W == 0 || info.format.pixel_shift_by_row.size() != H)
            throw std::invalid_argument("Invalid image dimensions or pixel shift count");
        row_shifts.reserve(H);
        for (const auto shift : info.format.pixel_shift_by_row) {
            const int64_t width = W;
            if (shift < -width || shift >= width)
                throw std::invalid_argument("Image pixel shift is outside [-width, width)");
            row_shifts.push_back(static_cast<size_t>((shift + width) % width));
        }

        image_msgs[ChanField::RANGE] = std::make_shared<sensor_msgs::msg::Image>();
        image_msgs[ChanField::SIGNAL] = std::make_shared<sensor_msgs::msg::Image>();
        image_msgs[ChanField::REFLECTIVITY] = std::make_shared<sensor_msgs::msg::Image>();
        image_msgs[ChanField::NEAR_IR] = std::make_shared<sensor_msgs::msg::Image>();
        if (info.num_returns() == 2) {
            image_msgs[ChanField::RANGE2] =
                std::make_shared<sensor_msgs::msg::Image>();
            image_msgs[ChanField::SIGNAL2] =
                std::make_shared<sensor_msgs::msg::Image>();
            image_msgs[ChanField::REFLECTIVITY2] =
                std::make_shared<sensor_msgs::msg::Image>();
        }

        for (auto it = image_msgs.begin(); it != image_msgs.end(); ++it) {
            init_image_msg(*it->second, H, W, frame);
        }

        mask = impl::load_mask<pixel_type>(mask_path, H, W);

        has_rgb_ =
            info.format.udp_profile_lidar == ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16 ||
            info.format.udp_profile_lidar == ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL;

        if (has_rgb_) {
            image_msgs[ChanField::RGB] = std::make_shared<sensor_msgs::msg::Image>();
            init_image_msg_rgb(*image_msgs[ChanField::RGB], H, W, frame);
        }
    }

   private:
    using pixel_type = uint16_t;
    const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();

    static void init_image_msg(sensor_msgs::msg::Image& msg, size_t H, size_t W,
                               const std::string& frame) {
        msg.width = W;
        msg.height = H;
        msg.step = W * sizeof(pixel_type);
        // TODO: allow choosing higher image encoding representation
        msg.encoding = sensor_msgs::image_encodings::MONO16;
        msg.data.resize(W * H * sizeof(pixel_type));
        msg.header.frame_id = frame;
    }

    static void init_image_msg_rgb(sensor_msgs::msg::Image& msg, size_t H, size_t W,
                                   const std::string& frame) {
        msg.width = W;
        msg.height = H;
        msg.step = W * 3 * sizeof(uint8_t);
        msg.encoding = sensor_msgs::image_encodings::RGB8;
        msg.data.resize(H * msg.step);
        msg.header.frame_id = frame;
    }

   private:
    void process(const ouster::sdk::core::LidarScan& lidar_scan, uint64_t,
                 const rclcpp::Time& msg_ts) {
        OutputType requested;
        // Unsupported channels (e.g. RGB on LEGACY) are absent from image_msgs,
        // so subscribers to an advertised but unavailable topic do no work.
        for (const auto& entry : image_msgs) {
            if (!demand_fn || demand_fn(entry.first)) requested.insert(entry);
        }
        if (requested.empty()) return;

        // Exposure/BUC state pauses while its channel family has no demand.
        // Join/rejoin can therefore briefly adjust exposure; always-process is
        // the compatibility mode for applications requiring continuous history.
        process_return(lidar_scan, 0, requested);
        if (info_.num_returns() == 2) process_return(lidar_scan, 1, requested);
        for (auto& entry : requested) entry.second->header.stamp = msg_ts;
        if (post_processing_fn) post_processing_fn(requested);
    }

    void write_gray(const std::string& channel,
                    const ouster::sdk::core::img_t<float>& pixels) {
        auto& message = *image_msgs.at(channel);
        auto output = Eigen::Map<ouster::sdk::core::img_t<pixel_type>>(
            reinterpret_cast<pixel_type*>(message.data.data()),
            info_.format.pixels_per_column, info_.format.columns_per_frame);
        output = (pixels * pixel_value_max).cast<pixel_type>();
        if (mask.size() != 0) output = output * mask;
    }

    template <typename T>
    void read_field(const ouster::sdk::core::LidarScan& scan,
                    const std::string& channel,
                    ouster::sdk::core::img_t<T>& buffer) {
        if (scan.has_field(channel)) {
            // Assign into the existing allocation, retaining the old integer
            // cast before conversion to float (notably reflectivity/near-IR).
            ouster::sdk::core::impl::visit_field(
                scan, channel, impl::read_and_cast(), buffer);
        } else {
            buffer.resize(info_.format.pixels_per_column, info_.format.columns_per_frame);
            buffer.setZero();
        }
    }

    void process_return(const ouster::sdk::core::LidarScan& lidar_scan,
                        int return_index, const OutputType& requested) {
        const bool first = return_index == 0;
        const auto range_channel = impl::scan_return(ChanField::RANGE, !first);
        const auto signal_channel = impl::scan_return(ChanField::SIGNAL, !first);
        const auto reflectivity_channel =
            impl::scan_return(ChanField::REFLECTIVITY, !first);
        const bool range_output = requested.count(range_channel) != 0;
        const bool signal_output = requested.count(signal_channel) != 0;
        const bool reflectivity_output = requested.count(reflectivity_channel) != 0;
        // Both returns share each AE object. A second-return subscriber still
        // needs the first return to update that object's state once per scan.
        const bool signal_needed = signal_output ||
            (first && requested.count(ChanField::SIGNAL2) != 0);
        const bool reflectivity_needed = reflectivity_output ||
            (first && requested.count(ChanField::REFLECTIVITY2) != 0);
        const bool nearir_output = first && requested.count(ChanField::NEAR_IR) != 0;
        const bool rgb_output = first && has_rgb_ && requested.count(ChanField::RGB) != 0;
        if (!(range_output || signal_needed || reflectivity_needed ||
              nearir_output || rgb_output)) return;

        const uint32_t H = info_.format.pixels_per_column;
        const uint32_t W = info_.format.columns_per_frame;
        const uint32_t* range = nullptr;
        pixel_type* range_pixels = nullptr;
        uint8_t* rgb_pixels = nullptr;
        if (range_output) {
            // The scan owns this immutable field for the entire synchronous
            // processor call; no staging copy is needed for range.
            range = lidar_scan.field<uint32_t>(range_channel).data();
            range_pixels = reinterpret_cast<pixel_type*>(
                image_msgs.at(range_channel)->data.data());
        }
        if (signal_needed) {
            read_field(lidar_scan, signal_channel, signal);
            signal_image.resize(H, W);
        }
        if (reflectivity_needed) {
            read_field(lidar_scan, reflectivity_channel, reflectivity);
            reflectivity_image.resize(H, W);
        }
        if (nearir_output) {
            read_field(lidar_scan, ChanField::NEAR_IR, near_ir);
            nearir_image.resize(H, W);
        }
        if (rgb_output) {
            read_field(lidar_scan, ChanField::R_U8, red);
            read_field(lidar_scan, ChanField::G_U8, green);
            read_field(lidar_scan, ChanField::B_U8, blue);
            rgb_pixels = image_msgs.at(ChanField::RGB)->data.data();
        }

        const auto process_pixel = [&](size_t target, size_t source) {
            if (range_output) {
                const auto r = (range[source] + 0b10) >> 2;
                range_pixels[target] = r > pixel_value_max ? 0 : r;
            }
            if (signal_needed) signal_image.data()[target] = signal.data()[source];
            if (reflectivity_needed)
                reflectivity_image.data()[target] = reflectivity.data()[source];
            if (nearir_output) nearir_image.data()[target] = near_ir.data()[source];
            if (rgb_output) {
                rgb_pixels[3 * target] = red.data()[source];
                rgb_pixels[3 * target + 1] = green.data()[source];
                rgb_pixels[3 * target + 2] = blue.data()[source];
            }
        };
        for (size_t u = 0; u < H; ++u) {
            const size_t row = u * W;
            const size_t split = row_shifts[u];
            // A destaggered row is two contiguous source spans. Keep the old
            // destination order without a modulo or wrap branch per pixel.
            const size_t tail = row + W - split;
            for (size_t v = 0; v < split; ++v) process_pixel(row + v, tail + v);
            for (size_t v = split; v < W; ++v) process_pixel(row + v, row + v - split);
        }

        if (signal_needed) {
            signal_ae.update(signal_image, first);
            if (signal_output) {
                signal_image = signal_image.sqrt();
                write_gray(signal_channel, signal_image);
            }
        }
        if (reflectivity_needed) {
            reflec_ae.update(reflectivity_image, first);
            if (reflectivity_output) write_gray(reflectivity_channel, reflectivity_image);
        }
        if (nearir_output) {
            nearir_buc.update(nearir_image);
            nearir_ae.update(nearir_image, first);
            nearir_image = nearir_image.sqrt();
            write_gray(ChanField::NEAR_IR, nearir_image);
        }
        if (mask.size() != 0) {
            if (range_output) {
                auto output = Eigen::Map<ouster::sdk::core::img_t<pixel_type>>(
                    range_pixels, H, W);
                output = output * mask;
            }
            if (rgb_output) {
                auto output = Eigen::TensorMap<ouster::sdk::core::rgb_img_t<uint8_t>>(
                    rgb_pixels, H, W, 3);
                using MaskMap = Eigen::TensorMap<
                    const Eigen::Tensor<pixel_type, 2, Eigen::RowMajor>>;
                MaskMap mask_tensor(mask.data(), H, W);
                const Eigen::array<Eigen::Index, 3> shape = {H, W, 1};
                const Eigen::array<Eigen::Index, 3> broadcast = {1, 1, 3};
                output *= mask_tensor.reshape(shape).broadcast(broadcast).cast<uint8_t>();
            }
        }
    }

   public:
    static LidarScanProcessor create(const ouster::sdk::core::SensorInfo& info,
                                     const std::string& frame,
                                     const std::string& mask_path,
                                     PostProcessingFn func, DemandFn demand = {}) {
        auto handler = std::make_shared<FrozenImageProcessor>(
            info, frame, mask_path, func, std::move(demand));
        return [handler](const ouster::sdk::core::LidarScan& lidar_scan, uint64_t scan_ts,
                         const rclcpp::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
    std::string frame;
    OutputType image_msgs;
    PostProcessingFn post_processing_fn;
    DemandFn demand_fn;
    ouster::sdk::core::SensorInfo info_;

    ouster::sdk::core::image::AutoExposure nearir_ae, signal_ae, reflec_ae;
    ouster::sdk::core::image::BeamUniformityCorrector nearir_buc;

    // Only demanded channel families resize these buffers. Retain allocations
    // between frames/returns; the native processing thread is their sole owner.
    std::vector<size_t> row_shifts;
    ouster::sdk::core::img_t<uint32_t> signal;
    ouster::sdk::core::img_t<uint16_t> reflectivity, near_ir;
    ouster::sdk::core::img_t<float> signal_image, reflectivity_image, nearir_image;
    ouster::sdk::core::img_t<uint8_t> red, green, blue;

    ouster::sdk::core::img_t<pixel_type> mask;
    bool has_rgb_ = false;
};

}  // namespace ouster_ros
