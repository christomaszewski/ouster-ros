/**
 * @file mask_image.cpp
 * @brief Reads the optional mask image (mask_path) and fits it to the scan size
 */

#include "mask_image.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

// The decoder is compiled into this file alone. STB_IMAGE_STATIC keeps every
// one of its symbols local, so a library loaded into the same process with its
// own copy of stb_image cannot collide with this one. The float formats are
// left out: a mask has no use for them.
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STBI_FAILURE_USERMSG
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_NO_PIC
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif
#include "../../thirdparty/stb/stb_image.h"
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace ouster_ros {
namespace impl {

namespace {

struct StbFree {
    void operator()(void* pixels) const { stbi_image_free(pixels); }
};

// 0.299 R + 0.587 G + 0.114 B in 14-bit fixed point, rounded to nearest.
int gray(int red, int green, int blue) {
    return (red * 4899 + green * 9617 + blue * 1868 + 8192) >> 14;
}

enum class Container { JPEG, PNM, OTHER };

// The two containers that need their own handling, told apart by content.
Container sniff(const std::string& path) {
    unsigned char magic[2] = {0, 0};
    std::unique_ptr<std::FILE, int (*)(std::FILE*)> file(
        std::fopen(path.c_str(), "rb"), &std::fclose);
    if (!file || std::fread(magic, 1, 2, file.get()) != 2)
        return Container::OTHER;
    if (magic[0] == 0xFF && magic[1] == 0xD8) return Container::JPEG;
    if (magic[0] == 'P' && (magic[1] == '5' || magic[1] == '6'))
        return Container::PNM;
    return Container::OTHER;
}

[[noreturn]] void fail(const std::string& path) {
    throw std::runtime_error(
        "Failed to load mask image from path: " + path + " (" +
        stbi_failure_reason() +
        "). Supported formats: PNG, JPEG, BMP, GIF, TGA, PSD, binary PGM/PPM.");
}

// Marks every pixel whose 8-bit gray value is not zero. `stride` is the
// distance in bytes from one sample to the next within a pixel.
MaskImage binarize(const unsigned char* pixel, int rows, int cols,
                   size_t channels, size_t stride) {
    MaskImage image;
    image.rows = static_cast<size_t>(rows);
    image.cols = static_cast<size_t>(cols);
    image.keep.resize(image.rows * image.cols);
    for (auto& keep : image.keep) {
        // one or two channels: gray (and alpha); three or four: RGB (and alpha)
        const int value = channels >= 3
            ? gray(pixel[0], pixel[stride], pixel[2 * stride]) : pixel[0];
        keep = value != 0;
        pixel += channels * stride;
    }
    return image;
}

std::vector<size_t> nearest_index(size_t source, size_t destination) {
    // Written as a reciprocal of the scale, not as source / destination: the
    // two round differently for some sizes and this is the form that selects
    // the same pixels as before.
    const double inverse_scale =
        1.0 / (static_cast<double>(destination) / static_cast<double>(source));
    std::vector<size_t> index(destination);
    for (size_t i = 0; i < destination; ++i) {
        const auto nearest =
            static_cast<size_t>(std::floor(static_cast<double>(i) * inverse_scale));
        index[i] = std::min(nearest, source - 1);
    }
    return index;
}

}  // namespace

MaskImage read_mask_image(const std::string& path) {
    const Container container = sniff(path);
    int cols = 0, rows = 0, file_channels = 0;

    if (container == Container::PNM && stbi_is_16_bit(path.c_str())) {
        // A 16-bit PGM/PPM sample is stored high byte first. The decoder hands
        // the bytes back as stored, and its own reduction to 8 bits would read
        // them in host order, taking the low byte on a little-endian machine.
        // Read the high byte of each sample from the stored order instead.
        std::unique_ptr<stbi_us, StbFree> samples(
            stbi_load_16(path.c_str(), &cols, &rows, &file_channels, 0));
        if (!samples) fail(path);
        return binarize(reinterpret_cast<const unsigned char*>(samples.get()),
                        rows, cols, static_cast<size_t>(file_channels), 2);
    }

    // A colour JPEG stores luma as its first component. Asking the decoder for
    // one channel returns that component untouched, which is the grayscale
    // image of the file; every other format is converted from its own colours.
    const int wanted_channels = container == Container::JPEG ? 1 : 0;
    std::unique_ptr<stbi_uc, StbFree> pixels(stbi_load(
        path.c_str(), &cols, &rows, &file_channels, wanted_channels));
    if (!pixels) fail(path);
    return binarize(pixels.get(), rows, cols,
                    static_cast<size_t>(wanted_channels != 0 ? wanted_channels
                                                             : file_channels),
                    1);
}

std::vector<uint8_t> resize_mask(const MaskImage& image, size_t height,
                                 size_t width) {
    if (image.rows == 0 || image.cols == 0 || height == 0 || width == 0)
        return {};
    if (image.rows == height && image.cols == width) return image.keep;

    const auto row_of = nearest_index(image.rows, height);
    const auto col_of = nearest_index(image.cols, width);
    std::vector<uint8_t> fitted(height * width);
    for (size_t row = 0; row < height; ++row) {
        const uint8_t* source = image.keep.data() + row_of[row] * image.cols;
        uint8_t* target = fitted.data() + row * width;
        for (size_t col = 0; col < width; ++col) target[col] = source[col_of[col]];
    }
    return fitted;
}

}  // namespace impl
}  // namespace ouster_ros
