#pragma once
#include "core-lib/Types.h"

namespace core {

/**
 * One 3-axis sample (raw sensor counts) plus the orientation derived from it.
 */
class AxesMeasurement {
public:
  /// Raw readings, one i16 per axis (6 bytes).
  struct AxesData {
    i16 data_x;
    i16 data_y;
    i16 data_z;
  };

  /// Orientation in degrees.
  struct Orientation {
    f32 roll;
    f32 pitch;
    f32 yaw;
  };

  AxesMeasurement() : axes{0, 0, 0}, orientation{0.0f, 0.0f, 0.0f} {}

  AxesData getAxes() const { return axes; }
  Orientation getOrientation() const { return orientation; }

  void set_x(i16 x) { axes.data_x = x; }
  void set_y(i16 y) { axes.data_y = y; }
  void set_z(i16 z) { axes.data_z = z; }

  void set_roll(f32 roll) { orientation.roll = roll; }
  void set_pitch(f32 pitch) { orientation.pitch = pitch; }
  void set_yaw(f32 yaw) { orientation.yaw = yaw; }

private:
  AxesData axes;
  Orientation orientation;
};

} // namespace core
