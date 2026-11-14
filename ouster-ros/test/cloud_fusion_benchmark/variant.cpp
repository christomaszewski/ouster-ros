#define PCL_NO_PRECOMPILE
#include "benchmark.hpp"
#include "point_cloud_compose.h"
#include "impl/cartesian.h"
#ifndef BASELINE
#include "cartesian_point_accessor.h"
#endif
namespace cloud_bench {
#ifdef BASELINE
class Production : public Variant {
    const Calibration& calibration;
    ouster_ros::CloudColumnWindow crop;
    ouster_ros::PointCloud2Builder<ouster_ros::Point> writer;
    core::PointCloudXYZf points;
    core::img_t<uint32_t> no_mask;
public:
    Production(const Calibration& c, cloud_test::Roi roi)
        : calibration(c), crop{int(roi.start), int(roi.width == c.width && roi.start == 0 ? 0 : roi.width)},
          writer(roi.width, c.height), points(c.width * c.height, 3) { writer.attach(output); }
    void run(const Input& input) override {
        auto range = input.scan.field<uint32_t>(core::ChanField::RANGE);
        if (crop.width)
            ouster::cartesianT_crop(points, range, calibration.direction, calibration.offset,
                input.minimum, input.maximum, std::numeric_limits<float>::quiet_NaN(),
                no_mask, crop.start, crop.width, 1, calibration.shifts, true);
        else
            ouster::cartesianT(points, range, calibration.direction, calibration.offset,
                input.minimum, input.maximum, std::numeric_limits<float>::quiet_NaN());
        ouster_ros::Point_LEGACY staging;
        ouster_ros::scan_to_cloud_f<ouster_ros::Profile_LEGACY.size(), ouster_ros::Profile_LEGACY>(
            writer, staging, points, input.scan_ts, input.scan, calibration.shifts, true, true, 1, crop);
    }
};
std::unique_ptr<Variant> make_baseline(const Calibration& c, cloud_test::Roi roi) {
    return std::make_unique<Production>(c, roi);
}
#else
template <size_t Tile, bool Neon>
class Candidate : public Variant {
    const Calibration& calibration;
    ouster_ros::CloudColumnWindow crop;
    ouster_ros::PointCloud2Builder<ouster_ros::Point> writer;
public:
    Candidate(const Calibration& c, cloud_test::Roi roi)
        : calibration(c), crop{int(roi.start), int(roi.width == c.width && roi.start == 0 ? 0 : roi.width)},
          writer(roi.width, c.height) { writer.attach(output); }
    void run(const Input& input) override {
        auto range = input.scan.field<uint32_t>(core::ChanField::RANGE);
        ouster_ros::CartesianPointRows<Tile, Neon> points(range.data(), range.size(),
            calibration.direction.data(), calibration.offset.data(), input.minimum, input.maximum);
        ouster_ros::Point_LEGACY staging;
        ouster_ros::scan_to_cloud_f<ouster_ros::Profile_LEGACY.size(), ouster_ros::Profile_LEGACY>(
            writer, staging, points, input.scan_ts, input.scan, calibration.shifts, true, true, 1, crop);
    }
};
std::unique_ptr<Variant> make_candidate(const Calibration& c, cloud_test::Roi roi, int variant) {
    switch (variant) {
        case 0: return std::make_unique<Candidate<0, false>>(c, roi);
        case 1: return std::make_unique<Candidate<32, false>>(c, roi);
        case 2: return std::make_unique<Candidate<64, false>>(c, roi);
        case 3: return std::make_unique<Candidate<128, false>>(c, roi);
        case 4: return std::make_unique<Candidate<32, true>>(c, roi);
        case 5: return std::make_unique<Candidate<64, true>>(c, roi);
        case 6: return std::make_unique<Candidate<128, true>>(c, roi);
        default: throw std::runtime_error("Unknown variant");
    }
}
#endif
}
