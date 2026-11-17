cmake_minimum_required(VERSION 3.20)
project(lidar_latency CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Release uses -O2 (CMake default is -O3) so baseline matches the spec.
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

option(LIDAR_BUILD_TESTS "Build tests" ON)
option(LIDAR_BUILD_BENCH "Build Google Benchmark micro-benchmarks" ON)

add_library(lidar
  src/io.cpp
  src/synthetic.cpp
  src/voxel.cpp
  src/ground.cpp
  src/cluster.cpp
  src/pipeline.cpp)
target_include_directories(lidar PUBLIC include)
target_compile_options(lidar PRIVATE -Wall -Wextra -Wpedantic)

add_executable(lidar_replay apps/replay.cpp)
target_link_libraries(lidar_replay PRIVATE lidar)

include(FetchContent)
if(LIDAR_BUILD_TESTS)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_executable(lidar_tests
    tests/test_io.cpp tests/test_voxel.cpp tests/test_ground.cpp
    tests/test_cluster.cpp tests/test_pipeline.cpp)
  target_link_libraries(lidar_tests PRIVATE lidar GTest::gtest_main)
  target_compile_definitions(lidar_tests PRIVATE LIDAR_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  include(GoogleTest)
  gtest_discover_tests(lidar_tests)
endif()

if(LIDAR_BUILD_BENCH)
  FetchContent_Declare(benchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(benchmark)
  add_executable(lidar_bench bench/bench_stages.cpp)
  target_link_libraries(lidar_bench PRIVATE lidar benchmark::benchmark)
endif()
