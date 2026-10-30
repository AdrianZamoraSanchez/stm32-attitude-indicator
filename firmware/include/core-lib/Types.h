#pragma once
#include <cstdint>

// Fixed-width type aliases used across the firmware.
// Portable on purpose: the same header is used by the host unit tests.

// Signed integers
using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

// Unsigned integers
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

// Floating point
static_assert(sizeof(float) == 4, "f32 requires a 32-bit float");
using f32 = float;

static_assert(sizeof(double) == 8, "f64 requires a 64-bit double");
using f64 = double;
