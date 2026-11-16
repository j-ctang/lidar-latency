#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "lidar/io.hpp"

using namespace lidar;

static std::string tmp_path(const char* name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

TEST(Io, LoadsFiveFloatRecords) {
  const std::string path = tmp_path("lidar_io_ok.pcd.bin");
  const float data[] = {1, 2, 3, 4, 0, -1, -2, -3, 0.5f, 31};
  { std::ofstream f(path, std::ios::binary); f.write(reinterpret_cast<const char*>(data), sizeof data); }
  PointCloud c = load_nuscenes_bin(path);
  ASSERT_EQ(c.size(), 2u);
  EXPECT_FLOAT_EQ(c[0].x, 1); EXPECT_FLOAT_EQ(c[0].intensity, 4);
  EXPECT_FLOAT_EQ(c[1].z, -3); EXPECT_FLOAT_EQ(c[1].intensity, 0.5f);
  std::remove(path.c_str());
}

TEST(Io, RejectsTruncatedFile) {
  const std::string path = tmp_path("lidar_io_bad.pcd.bin");
  const float data[] = {1, 2, 3};
  { std::ofstream f(path, std::ios::binary); f.write(reinterpret_cast<const char*>(data), sizeof data); }
  EXPECT_THROW(load_nuscenes_bin(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Io, MissingFileThrows) {
  EXPECT_THROW(load_nuscenes_bin("/nonexistent/x.pcd.bin"), std::runtime_error);
}
