#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace {

// A 4x2 binary PGM: 0 9 0 9 / 9 9 0 0
std::string write_pgm() {
    const std::string path = testing::TempDir() + "load_mask_test_" +
                             std::to_string(::getpid()) + ".pgm";
    std::ofstream file(path, std::ios::binary);
    file << "P5\n4 2\n255\n";
    const char pixels[] = {0, 9, 0, 9, 9, 9, 0, 0};
    file.write(pixels, sizeof(pixels));
    return path;
}

}  // namespace

TEST(LoadMask, EmptyPathMeansNoMask) {
    EXPECT_EQ(ouster_ros::impl::load_mask<uint32_t>("", 2, 4).size(), 0);
}

TEST(LoadMask, ReturnsARowMajorImageOfZerosAndOnes) {
    const std::string path = write_pgm();
    const auto mask = ouster_ros::impl::load_mask<uint32_t>(path, 2, 4);
    std::remove(path.c_str());

    ASSERT_EQ(mask.rows(), 2);
    ASSERT_EQ(mask.cols(), 4);
    const uint32_t expected[2][4] = {{0, 1, 0, 1}, {1, 1, 0, 0}};
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 4; ++col)
            EXPECT_EQ(mask(row, col), expected[row][col])
                << "row " << row << " col " << col;
}

TEST(LoadMask, FitsTheImageToTheScan) {
    const std::string path = write_pgm();
    const auto mask = ouster_ros::impl::load_mask<uint16_t>(path, 2, 8);
    std::remove(path.c_str());

    ASSERT_EQ(mask.rows(), 2);
    ASSERT_EQ(mask.cols(), 8);
    const uint16_t expected[2][8] = {{0, 0, 1, 1, 0, 0, 1, 1},
                                     {1, 1, 1, 1, 0, 0, 0, 0}};
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 8; ++col)
            EXPECT_EQ(mask(row, col), expected[row][col])
                << "row " << row << " col " << col;
}

TEST(LoadMask, ReportsAFileItCannotRead) {
    EXPECT_THROW(ouster_ros::impl::load_mask<uint32_t>(
                     testing::TempDir() + "no_such_mask.png", 2, 4),
                 std::runtime_error);
}
