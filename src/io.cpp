#include "lidar/io.hpp"

#include <fstream>
#include <stdexcept>

namespace lidar {

PointCloud load_nuscenes_bin(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) throw std::runtime_error("cannot open " + path);
  const auto bytes = static_cast<std::size_t>(f.tellg());
  constexpr std::size_t kStride = 5 * sizeof(float);
  if (bytes % kStride != 0)
    throw std::runtime_error("size not a multiple of 20 bytes: " + path);
  std::vector<float> raw(bytes / sizeof(float));
  f.seekg(0);
  if (!f.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(bytes)))
    throw std::runtime_error("short read: " + path);
  PointCloud cloud(bytes / kStride);
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const float* r = &raw[i * 5];
    cloud[i] = {r[0], r[1], r[2], r[3]};
  }
  return cloud;
}

}  // namespace lidar
