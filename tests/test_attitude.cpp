#include "core-lib/Attitude.h"

#include <cmath>
#include <gtest/gtest.h>

using core::attitude::ComplementaryFilter;
using core::attitude::Euler;
using core::attitude::from_accel;
using core::attitude::MountCompensation;
using core::attitude::to_vehicle_frame;
using core::attitude::wrap_180;

namespace {

constexpr f32 kTol = 0.01f;

f32 deg_sin(f32 deg) { return std::sin(deg * core::attitude::DEG_TO_RAD); }
f32 deg_cos(f32 deg) { return std::cos(deg * core::attitude::DEG_TO_RAD); }

} // namespace

// ---- Accelerometer attitude ------------------------------------------------

TEST(FromAccel, FlatIsLevel) {
  const Euler e = from_accel(0.0f, 0.0f, 16384.0f);
  EXPECT_NEAR(e.roll, 0.0f, kTol);
  EXPECT_NEAR(e.pitch, 0.0f, kTol);
}

TEST(FromAccel, RollAroundX) {
  // Rotating +30 deg about X moves gravity into +Y
  const Euler e = from_accel(0.0f, deg_sin(30.0f), deg_cos(30.0f));
  EXPECT_NEAR(e.roll, 30.0f, kTol);
  EXPECT_NEAR(e.pitch, 0.0f, kTol);
}

TEST(FromAccel, PitchAroundY) {
  // Rotating +20 deg about Y moves gravity into -X
  const Euler e = from_accel(-deg_sin(20.0f), 0.0f, deg_cos(20.0f));
  EXPECT_NEAR(e.roll, 0.0f, kTol);
  EXPECT_NEAR(e.pitch, 20.0f, kTol);
}

TEST(FromAccel, ScaleDoesNotMatter) {
  const Euler a = from_accel(-0.3f, 0.2f, 0.9f);
  const Euler b = from_accel(-3000.0f, 2000.0f, 9000.0f);
  EXPECT_NEAR(a.roll, b.roll, kTol);
  EXPECT_NEAR(a.pitch, b.pitch, kTol);
}

TEST(Wrap180, KeepsRange) {
  EXPECT_NEAR(wrap_180(190.0f), -170.0f, kTol);
  EXPECT_NEAR(wrap_180(-190.0f), 170.0f, kTol);
  EXPECT_NEAR(wrap_180(180.0f), -180.0f, kTol);
  EXPECT_NEAR(wrap_180(45.0f), 45.0f, kTol);
}

// ---- Complementary filter --------------------------------------------------

TEST(ComplementaryFilter, FirstUpdateTakesAccelerometer) {
  ComplementaryFilter f(0.98f);
  const Euler e = f.update({12.0f, -5.0f}, 50.0f, 50.0f, 0.0f, 0.01f);
  EXPECT_NEAR(e.roll, 12.0f, kTol);
  EXPECT_NEAR(e.pitch, -5.0f, kTol);
}

TEST(ComplementaryFilter, ConvergesToAccelerometerWithoutRotation) {
  ComplementaryFilter f(0.98f);
  f.reset({0.0f, 0.0f});

  Euler e{};
  for (int i = 0; i < 500; ++i) // 5 s at 100 Hz, tau ~= 0.5 s
    e = f.update({10.0f, -4.0f}, 0.0f, 0.0f, 0.0f, 0.01f);

  EXPECT_NEAR(e.roll, 10.0f, 0.05f);
  EXPECT_NEAR(e.pitch, -4.0f, 0.05f);
}

TEST(ComplementaryFilter, FollowsGyroInTheShortTerm) {
  // Accelerometer stuck at level (e.g. vibration-free but lagging): a fast
  // 100 deg/s roll for 0.1 s must still show up almost entirely.
  ComplementaryFilter f(0.98f);
  f.reset({0.0f, 0.0f});

  Euler e{};
  for (int i = 0; i < 10; ++i)
    e = f.update({0.0f, 0.0f}, 100.0f, 0.0f, 0.0f, 0.01f);

  EXPECT_GT(e.roll, 8.0f);
  EXPECT_LT(e.roll, 10.0f);
  EXPECT_NEAR(e.pitch, 0.0f, kTol);
}

TEST(ComplementaryFilter, RejectsAccelerometerSpikes) {
  ComplementaryFilter f(0.98f);
  f.reset({0.0f, 0.0f});

  // One corrupted accelerometer sample (a knock) only moves 2% of the way
  const Euler e = f.update({45.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 0.01f);
  EXPECT_NEAR(e.roll, 0.9f, kTol);
}

TEST(ComplementaryFilter, RollDoesNotJumpAcross180) {
  ComplementaryFilter f(0.5f);
  f.reset({179.0f, 0.0f});

  // Accelerometer says -179 (2 deg away, across the wrap), not 358 deg away
  const Euler e = f.update({-179.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 0.01f);
  EXPECT_NEAR(std::fabs(e.roll), 180.0f, 0.01f);
}

TEST(ComplementaryFilter, AlphaZeroIsAccelerometerOnly) {
  ComplementaryFilter f(0.0f);
  f.reset({0.0f, 0.0f});
  const Euler e = f.update({7.0f, 3.0f}, 200.0f, 200.0f, 0.0f, 0.01f);
  EXPECT_NEAR(e.roll, 7.0f, kTol);
  EXPECT_NEAR(e.pitch, 3.0f, kTol);
}

TEST(ComplementaryFilter, InvalidDtResets) {
  ComplementaryFilter f(0.98f);
  f.reset({0.0f, 0.0f});
  const Euler e = f.update({5.0f, 5.0f}, 100.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(e.roll, 5.0f, kTol);
}

// ---- Mount compensation ----------------------------------------------------

TEST(MountCompensation, IdentityLeavesAnglesAlone) {
  const Euler v = to_vehicle_frame({10.0f, 20.0f}, {false, 1.0f, 1.0f});
  EXPECT_FLOAT_EQ(v.roll, 10.0f);
  EXPECT_FLOAT_EQ(v.pitch, 20.0f);
}

TEST(MountCompensation, SwapAndInvert) {
  // Current board mounting: vehicle roll = -sensor pitch,
  //                         vehicle pitch = -sensor roll
  const Euler v = to_vehicle_frame({10.0f, 20.0f}, {true, -1.0f, -1.0f});
  EXPECT_FLOAT_EQ(v.roll, -20.0f);
  EXPECT_FLOAT_EQ(v.pitch, -10.0f);
}
