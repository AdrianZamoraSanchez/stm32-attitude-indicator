#pragma once
#include "core-lib/Types.h"
#include <cmath>

// Attitude (roll/pitch) estimation from a 6-axis IMU.
// Header-only and hardware-independent, so it is unit tested on the host.

namespace core::attitude {

constexpr f32 RAD_TO_DEG = 57.2957795f;
constexpr f32 DEG_TO_RAD = 0.0174532925f;

/// Roll and pitch in degrees.
struct Euler {
  f32 roll;
  f32 pitch;
};

/// Wrap an angle to [-180, 180).
inline f32 wrap_180(f32 deg) {
  while (deg >= 180.0f)
    deg -= 360.0f;
  while (deg < -180.0f)
    deg += 360.0f;
  return deg;
}

/**
 * Roll and pitch from the gravity vector measured by an accelerometer.
 * Units do not matter (only ratios are used), so raw counts are fine.
 * Only valid while the sensor is not accelerating: any linear acceleration
 * or vibration is read as tilt.
 */
inline Euler from_accel(f32 ax, f32 ay, f32 az) {
  return {std::atan2(ay, az) * RAD_TO_DEG,
          std::atan2(-ax, std::sqrt((ay * ay) + (az * az))) * RAD_TO_DEG};
}

/**
 * Complementary filter: fuses gyro and accelerometer estimates.
 *
 * The gyro is integrated for fast, smooth response (but it drifts), and the
 * result is pulled slowly towards the accelerometer estimate (noisy, but
 * drift free):
 *
 *   angle = alpha * (angle + rate * dt) + (1 - alpha) * accel_angle
 *
 * With alpha = 0.98 at 100 Hz the time constant is
 * tau = alpha * dt / (1 - alpha) ~= 0.5 s.
 * alpha = 0 means accelerometer only, alpha = 1 means gyro only.
 */
class ComplementaryFilter {
public:
  explicit constexpr ComplementaryFilter(f32 alpha) : alpha_(alpha) {}

  /// Start from a known attitude (normally the first accelerometer reading).
  void reset(const Euler &attitude) {
    estimate_ = attitude;
    initialized_ = true;
  }

  /**
   * @param accel     attitude computed from the accelerometer (degrees)
   * @param gx,gy,gz  angular rates in the sensor frame (degrees/second)
   * @param dt        time since the previous update (seconds)
   * @return the new estimate
   */
  Euler update(const Euler &accel, f32 gx, f32 gy, f32 gz, f32 dt) {
    if (!initialized_ || dt <= 0.0f) {
      reset(accel);
      return estimate_;
    }

    // Body rates -> Euler angle rates (roll/pitch kinematics)
    const f32 phi = estimate_.roll * DEG_TO_RAD;
    const f32 theta = estimate_.pitch * DEG_TO_RAD;

    const f32 sin_phi = std::sin(phi);
    const f32 cos_phi = std::cos(phi);
    const f32 cos_theta = std::cos(theta);

    // tan(theta) is singular at +/-90 deg pitch (gimbal lock): skip the
    // coupling term there instead of injecting a huge rate.
    const f32 coupling =
        (std::fabs(cos_theta) > 1.0e-3f) ? std::tan(theta) : 0.0f;

    const f32 roll_rate = gx + ((sin_phi * gy) + (cos_phi * gz)) * coupling;
    const f32 pitch_rate = (cos_phi * gy) - (sin_phi * gz);

    const f32 roll_pred = estimate_.roll + (roll_rate * dt);
    const f32 pitch_pred = estimate_.pitch + (pitch_rate * dt);

    // Blend along the shortest angular difference so roll does not jump
    // when it crosses +/-180 deg.
    const f32 k = 1.0f - alpha_;
    estimate_.roll =
        wrap_180(roll_pred + (k * wrap_180(accel.roll - roll_pred)));
    estimate_.pitch = pitch_pred + (k * (accel.pitch - pitch_pred));

    return estimate_;
  }

  Euler estimate() const { return estimate_; }

private:
  f32 alpha_;
  Euler estimate_{0.0f, 0.0f};
  bool initialized_ = false;
};

/**
 * Compensation for how the sensor board is mounted on the vehicle.
 * Maps the attitude in the sensor frame to the vehicle frame by optionally
 * swapping roll and pitch and flipping their signs.
 */
struct MountCompensation {
  bool swap_roll_pitch;
  f32 roll_sign;  // +1 or -1
  f32 pitch_sign; // +1 or -1
};

inline Euler to_vehicle_frame(const Euler &sensor,
                              const MountCompensation &mount) {
  Euler vehicle =
      mount.swap_roll_pitch ? Euler{sensor.pitch, sensor.roll} : sensor;

  vehicle.roll *= mount.roll_sign;
  vehicle.pitch *= mount.pitch_sign;

  return vehicle;
}

} // namespace core::attitude
