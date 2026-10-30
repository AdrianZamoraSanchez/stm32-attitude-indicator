# Host unit tests (x86/x64, normal compiler). No hardware needed.
#   cmake -B build -S . && cmake --build build && ctest --test-dir build

cmake_minimum_required(VERSION 3.20)
project(attitude_indicator_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(unit_tests
  test_string_utils.cpp
  test_attitude.cpp
  test_mpu6050.cpp
)

target_include_directories(unit_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/include
)

target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(unit_tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(unit_tests)
