cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
  message(FATAL_ERROR
    "Cross-compilation only. Configure with:\n"
    "  cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake\n"
    "Host unit tests live in ../tests.")
endif()

project(stm32_attitude_indicator LANGUAGES C CXX ASM)

# C/C++ version
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Debug: no optimization, best for stepping with GDB
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Release
set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

add_subdirectory(stm32f4xx)
add_subdirectory(src)
add_subdirectory(app)
