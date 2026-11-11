/**
 * @file mask_image.h
 * @brief Reads the optional mask image (mask_path) and fits it to the scan size
 *
 * Nothing here depends on ROS, so the functions can be tested on their own.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ouster_ros {
namespace impl {

/**
 * A mask as read from disk: one value per pixel in row-major order, 0 where
 * the pixel is black and 1 everywhere else.
 */
struct MaskImage {
    size_t rows{0};
    size_t cols{0};
    std::vector<uint8_t> keep;
};

/**
 * Reads a mask image. PNG, JPEG, BMP, GIF, TGA, PSD and binary PGM/PPM files
 * are accepted; the format is taken from the file's content, not its name.
 *
 * A pixel is black when its 8-bit grayscale value is 0. Colour pixels are
 * converted with the usual 0.299 R + 0.587 G + 0.114 B weights, rounded; a
 * colour JPEG contributes its stored luma. Alpha is ignored, and 16-bit
 * samples are reduced to their high byte.
 *
 * @param[in] path path to the image file
 * @return the decoded mask
 * @throws std::runtime_error when the file cannot be read or decoded
 */
MaskImage read_mask_image(const std::string& path);

/**
 * Fits a mask to height x width by nearest-neighbour sampling: destination
 * index i reads source index min(floor(i * source / destination), source - 1)
 * along each axis.
 *
 * @param[in] image the mask to sample
 * @param[in] height number of rows wanted
 * @param[in] width number of columns wanted
 * @return height * width values of 0 or 1 in row-major order
 */
std::vector<uint8_t> resize_mask(const MaskImage& image, size_t height,
                                 size_t width);

}  // namespace impl
}  // namespace ouster_ros
