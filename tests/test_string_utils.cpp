#include "core-lib/StringUtils.h"

#include <gtest/gtest.h>
#include <limits>
#include <string>

namespace {

std::string fmt(f32 value, u8 decimals = 2, u32 size = 32) {
  char buffer[32] = {};
  const u32 n = core::f32_to_string(value, buffer, size, decimals);
  return (n == 0U) ? std::string("<fail>") : std::string(buffer, n);
}

} // namespace

TEST(U32ToString, Zero) {
  char buffer[4];
  EXPECT_EQ(core::u32_to_string(0U, buffer, sizeof(buffer)), 1U);
  EXPECT_STREQ(buffer, "0");
}

TEST(U32ToString, MaxValue) {
  char buffer[11];
  EXPECT_EQ(core::u32_to_string(4294967295U, buffer, sizeof(buffer)), 10U);
  EXPECT_STREQ(buffer, "4294967295");
}

TEST(U32ToString, BufferTooSmallFails) {
  char buffer[3];
  EXPECT_EQ(core::u32_to_string(123U, buffer, sizeof(buffer)), 0U);
}

TEST(F32ToString, PositiveAndNegative) {
  EXPECT_EQ(fmt(12.34f), "12.34");
  EXPECT_EQ(fmt(-3.14159f), "-3.14");
}

TEST(F32ToString, KeepsLeadingZerosInFraction) {
  EXPECT_EQ(fmt(1.05f), "1.05");
  EXPECT_EQ(fmt(0.5f, 3), "0.500");
}

TEST(F32ToString, RoundsAndCarriesIntoIntegerPart) {
  EXPECT_EQ(fmt(1.999f), "2.00");
  EXPECT_EQ(fmt(-0.004f), "0.00"); // no "-0.00"
}

TEST(F32ToString, ZeroDecimals) { EXPECT_EQ(fmt(41.6f, 0), "42"); }

TEST(F32ToString, DecimalsAreClampedToSix) {
  EXPECT_EQ(fmt(1.25f, 9), "1.250000");
}

TEST(F32ToString, RejectsNotRepresentableValues) {
  EXPECT_EQ(fmt(std::numeric_limits<f32>::quiet_NaN()), "<fail>");
  EXPECT_EQ(fmt(std::numeric_limits<f32>::infinity()), "<fail>");
  EXPECT_EQ(fmt(1.0e9f), "<fail>"); // 1e9 * 100 does not fit in a u32
}

TEST(F32ToString, BufferTooSmallFails) {
  EXPECT_EQ(fmt(-123.45f, 2, 7), "<fail>"); // needs 7 chars + terminator
  EXPECT_EQ(fmt(-123.45f, 2, 8), "-123.45");
}
