#pragma once

#include <ouster/types.h>
#include <stdexcept>
#include <vector>

namespace ouster_ros {

// Columns in the original organized/destaggered grid. width == 0 disables
// cropping. PointCloud2 has no column-origin field; consumers recover this
// stable mapping from sensor metadata and the configured destagger/row step.
struct CloudColumnWindow {
    int start = 0;
    int width = 0;
};

inline CloudColumnWindow cloud_column_window(
    const ouster::sdk::core::SensorInfo& info, bool organized, bool destagger,
    int rows_step, bool crop) {
    if (!crop) return {};
    if (!organized)
        throw std::invalid_argument("organized_cloud_crop requires organized=true");
    const int w = info.format.columns_per_frame;
    const int h = info.format.pixels_per_column;
    const auto first = info.format.column_window.first;
    const auto last = info.format.column_window.second;
    if (w <= 0 || h <= 0 || rows_step <= 0 || h % rows_step != 0 ||
        first < 0 || first >= w || last < 0 || last >= w ||
        static_cast<int>(info.format.pixel_shift_by_row.size()) != h)
        throw std::invalid_argument("invalid organized cloud crop metadata or row step");

    const int columns = (last - first + w) % w + 1;
    std::vector<bool> included(w, false);
    for (int row = 0; row < h; row += rows_step) {
        const int shift = info.format.pixel_shift_by_row[row];
        if (shift < -w || shift >= w)
            throw std::invalid_argument("invalid organized cloud crop row shift");
        const int normalized = destagger ? (shift + w) % w : 0;
        int col = (first + normalized) % w;
        for (int n = 0; n < columns; ++n) {
            included[col] = true;
            if (++col == w) col = 0;
        }
    }

    // The minimal circular bounding interval is the complement of the longest
    // empty gap. Scan from the first included column for deterministic ties.
    int anchor = 0;
    while (!included[anchor]) ++anchor;
    int gap = 0, longest_gap = 0, start = 0;
    for (int n = 1; n <= w; ++n) {
        const int col = (anchor + n) % w;
        if (!included[col]) {
            ++gap;
        } else {
            if (gap > longest_gap) {
                longest_gap = gap;
                start = col;
            }
            gap = 0;
        }
    }
    return {start, w - longest_gap};
}

}  // namespace ouster_ros
