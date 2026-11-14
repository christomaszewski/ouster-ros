#pragma once

#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <Eigen/Core>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace ouster_ros {

// The input pointers borrow the current scan and the processor-owned LUT/mask.
// Access is synchronous on the existing scan-processing thread. Tile variants
// are bounded experiments; zero selects scalar fusion without XYZ scratch.
template <size_t TileSize = 0, bool Neon = false>
class CartesianPointRows {
   public:
    CartesianPointRows(const uint32_t* range, size_t count, const float* direction,
                       const float* offset, uint32_t minimum, uint32_t maximum,
                       const uint32_t* mask = nullptr)
        : range_(range), count_(count), direction_(direction), offset_(offset),
          minimum_(minimum), maximum_(maximum), mask_(mask) {}

    Eigen::RowVector3f row(size_t index) const {
        if constexpr (TileSize == 0) {
            Eigen::RowVector3f result;
            calculate(index, result.data());
            return result;
        } else {
            const size_t base = index / TileSize * TileSize;
            if (base != tile_base_) fill(base);
            return Eigen::Map<const Eigen::RowVector3f>(tile_.data() + (index - base) * 3);
        }
    }

   private:
    void calculate(size_t index, float* xyz) const {
        const uint32_t r = mask_ ? range_[index] * mask_[index] : range_[index];
        const auto n = index * 3;
        if (r <= minimum_ || r >= maximum_) {
            xyz[0] = xyz[1] = xyz[2] = std::numeric_limits<float>::quiet_NaN();
        } else {
            // Preserve cartesianT's uint32-to-float conversion and FMA policy.
            if constexpr (Neon) {
                // Keep a partial SIMD tile on the same single-rounding policy
                // as its vector lanes, including under sanitizer instrumentation.
                xyz[0] = std::fma(static_cast<float>(r), direction_[n], offset_[n]);
                xyz[1] = std::fma(static_cast<float>(r), direction_[n + 1], offset_[n + 1]);
                xyz[2] = std::fma(static_cast<float>(r), direction_[n + 2], offset_[n + 2]);
            } else {
                xyz[0] = r * direction_[n] + offset_[n];
                xyz[1] = r * direction_[n + 1] + offset_[n + 1];
                xyz[2] = r * direction_[n + 2] + offset_[n + 2];
            }
        }
    }

    void fill(size_t base) const {
        const size_t end = std::min(count_, base + TileSize);
        size_t i = base;
#if defined(__aarch64__)
        if constexpr (Neon) {
            for (; i + 4 <= end; i += 4) {
                auto r = vld1q_u32(range_ + i);
                if (mask_) r = vmulq_u32(r, vld1q_u32(mask_ + i));
                const auto valid = vandq_u32(vcgtq_u32(r, vdupq_n_u32(minimum_)),
                                             vcltq_u32(r, vdupq_n_u32(maximum_)));
                const auto distance = vcvtq_f32_u32(r);
                const auto direction = vld3q_f32(direction_ + i * 3);
                const auto offset = vld3q_f32(offset_ + i * 3);
                const auto invalid = vdupq_n_f32(std::numeric_limits<float>::quiet_NaN());
                float32x4x3_t xyz;
                for (int axis = 0; axis < 3; ++axis)
                    xyz.val[axis] = vbslq_f32(valid,
                        vfmaq_f32(offset.val[axis], distance, direction.val[axis]), invalid);
                vst3q_f32(tile_.data() + (i - base) * 3, xyz);
            }
        }
#endif
        for (; i < end; ++i) calculate(i, tile_.data() + (i - base) * 3);
        tile_base_ = base;
    }

    const uint32_t* range_;
    size_t count_;
    const float* direction_;
    const float* offset_;
    uint32_t minimum_, maximum_;
    const uint32_t* mask_;
    mutable size_t tile_base_ = std::numeric_limits<size_t>::max();
    mutable std::array<float, (TileSize ? TileSize : 1) * 3> tile_;
};

using CartesianPointAccessor = CartesianPointRows<>;

}  // namespace ouster_ros
