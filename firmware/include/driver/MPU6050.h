#pragma once

#include "HAL/HAL_I2C.h"
#include "core-lib/AxesMeasurement.h"
#include <cmath>

namespace drivers::MPU6050 {

constexpr u8 MPU6050_ADDR = 0x68;

// Identification
constexpr u8 WHO_AM_I = 0x75;
constexpr u8 WHO_AM_I_MPU6050 = 0x68;
constexpr u8 WHO_AM_I_MPU6500 = 0x70;

// Power / reset
constexpr u8 PWR_MGMT_1 = 0x6B;
constexpr u8 PWR_MGMT_2 = 0x6C;
constexpr u8 SIGNAL_RESET = 0x68;
constexpr u8 USER_CTRL = 0x6A;

// Accelerometers
constexpr u8 ACCEL_START = 0x3B;
constexpr u8 ACCEL_X_HI = 0x3B;
constexpr u8 ACCEL_X_LO = 0x3C;

constexpr u8 ACCEL_Y_HI = 0x3D;
constexpr u8 ACCEL_Y_LO = 0x3E;

constexpr u8 ACCEL_Z_HI = 0x3F;
constexpr u8 ACCEL_Z_LO = 0x40;

// Temperature
constexpr u8 TEMP_X_HI = 0x41;
constexpr u8 TEMP_X_LO = 0x42;

// Gyroscopes
constexpr u8 GYRO_START = 0x43;
constexpr u8 GYRO_X_HI = 0x43;
constexpr u8 GYRO_X_LO = 0x44;

constexpr u8 GYRO_Y_HI = 0x45;
constexpr u8 GYRO_Y_LO = 0x46;

constexpr u8 GYRO_Z_HI = 0x47;
constexpr u8 GYRO_Z_LO = 0x48;

// Config
constexpr u8 GYRO_CONFIG = 0x1B;
constexpr u8 ACCEL_CONFIG = 0x1C;
constexpr u8 SMPLRT_DIV = 0x19;
constexpr u8 MPU_CONFIG = 0x1A;

// Config values
constexpr u8 PWR_RESET = 0x80;
constexpr u8 PWR_WAKE_PLL_XGYRO = 0x01;
constexpr u8 ALL_AXES_ENABLED = 0x00;

constexpr u8 DLPF_44HZ = 0x03;
constexpr u8 DLPF_20HZ = 0x04;
constexpr u8 DLPF_10HZ = 0x05;
constexpr u8 DLPF_5HZ = 0x06;

constexpr u8 SAMPLE_RATE_100HZ = 0x09; // 1 kHz / (9 + 1)
constexpr u8 SAMPLE_RATE_125HZ = 0x07; // 1 kHz / (7 + 1)

constexpr u8 ACCEL_FS_2G = 0x00;
constexpr u8 GYRO_FS_250DPS = 0x00;

// Scale factors for selected config
constexpr f32 ACCEL_2G_LSB_PER_G = 16384.0f;
constexpr f32 GYRO_250DPS_LSB_PER_DPS = 131.0f;

struct AxesBias {
  i32 x;
  i32 y;
  i32 z;
};

template <typename I2C, typename Time> class MPU6050 {
public:
  explicit MPU6050(I2C &i2c, Time &time)
      : i2c_(i2c), time_(time), gyro_bias{0, 0, 0} {}

  bool init();

  bool read_reg(u8 reg, u8 &value);
  bool write_reg(u8 reg, u8 value);

  bool read_i16(u8 reg, i16 &value);
  bool read_bytes(u8 reg, u8 *data, u8 len);

  bool read_all_data(u8 reg, core::AxesMeasurement &value);

  bool read_accel(core::AxesMeasurement &value);
  bool read_gyro_raw(core::AxesMeasurement &value);
  bool read_gyro(core::AxesMeasurement &value);

  bool calibrate_gyro(u16 samples = 500, u16 delay_ms = 2);

  static f32 accel_raw_to_g(i16 raw);
  static f32 gyro_raw_to_dps(i16 raw);

  bool read_accel_orientation(core::AxesMeasurement &value);

private:
  static i16 make_i16(u8 hi, u8 lo);
  static i16 saturate_i32_to_i16(i32 value);

private:
  I2C &i2c_;
  Time &time_;
  AxesBias gyro_bias;
};

template <typename I2C, typename Time>
i16 MPU6050<I2C, Time>::make_i16(u8 hi, u8 lo) {
  const u16 raw = (static_cast<u16>(hi) << 8) | static_cast<u16>(lo);
  return static_cast<i16>(raw);
}

template <typename I2C, typename Time>
i16 MPU6050<I2C, Time>::saturate_i32_to_i16(i32 value) {
  if (value > 32767)
    return 32767;

  if (value < -32768)
    return -32768;

  return static_cast<i16>(value);
}

template <typename I2C, typename Time>
f32 MPU6050<I2C, Time>::accel_raw_to_g(i16 raw) {
  return static_cast<f32>(raw) / ACCEL_2G_LSB_PER_G;
}

template <typename I2C, typename Time>
f32 MPU6050<I2C, Time>::gyro_raw_to_dps(i16 raw) {
  return static_cast<f32>(raw) / GYRO_250DPS_LSB_PER_DPS;
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_reg(u8 reg, u8 &value) {
  return read_bytes(reg, &value, 1);
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::write_reg(u8 reg, u8 value) {
  u8 buf[2] = {reg, value};

  return i2c_.write(MPU6050_ADDR, buf, 2) == HAL::I2C_Error::None;
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_i16(u8 reg, i16 &value) {
  u8 data[2] = {0};

  if (!read_bytes(reg, data, sizeof(data)))
    return false;

  value = make_i16(data[0], data[1]);
  return true;
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_bytes(u8 reg, u8 *data, u8 len) {
  if (data == nullptr || len == 0)
    return false;

  auto err = i2c_.start(MPU6050_ADDR, false, false);
  if (err != HAL::I2C_Error::None)
    return false;

  err = i2c_.write_raw(&reg, 1);
  if (err != HAL::I2C_Error::None) {
    i2c_.stop();
    return false;
  }

  err = i2c_.start(MPU6050_ADDR, true, true);
  if (err != HAL::I2C_Error::None) {
    i2c_.stop();
    return false;
  }

  err = i2c_.read_raw(data, len);
  if (err != HAL::I2C_Error::None) {
    i2c_.stop();
    return false;
  }

  i2c_.stop();
  return true;
}

// Get a complete measurement from GYRO or ACCEL.
// Pass ACCEL_START or GYRO_START.
template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_all_data(u8 reg, core::AxesMeasurement &value) {
  u8 data[6] = {0};

  if (!read_bytes(reg, data, sizeof(data)))
    return false;

  const i16 raw_x = make_i16(data[0], data[1]);
  const i16 raw_y = make_i16(data[2], data[3]);
  const i16 raw_z = make_i16(data[4], data[5]);

  value.set_x(raw_x);
  value.set_y(raw_y);
  value.set_z(raw_z);

  return true;
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_accel_orientation(core::AxesMeasurement &value) {
  if (!read_accel(value))
    return false;

  const f32 ax = static_cast<f32>(value.getAxes().data_x);
  const f32 ay = static_cast<f32>(value.getAxes().data_y);
  const f32 az = static_cast<f32>(value.getAxes().data_z);

  constexpr f32 RAD_TO_DEG = 57.2957795f;

  const f32 roll = atan2f(ay, az) * RAD_TO_DEG;
  const f32 pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;

  value.set_roll(roll);
  value.set_pitch(pitch);

  return true;
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_accel(core::AxesMeasurement &value) {
  return read_all_data(ACCEL_START, value);
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_gyro_raw(core::AxesMeasurement &value) {
  return read_all_data(GYRO_START, value);
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::read_gyro(core::AxesMeasurement &value) {
  core::AxesMeasurement raw;

  if (!read_gyro_raw(raw))
    return false;

  const i32 x = static_cast<i32>(raw.getAxes().data_x) - gyro_bias.x;
  const i32 y = static_cast<i32>(raw.getAxes().data_y) - gyro_bias.y;
  const i32 z = static_cast<i32>(raw.getAxes().data_z) - gyro_bias.z;

  value.set_x(saturate_i32_to_i16(x));
  value.set_y(saturate_i32_to_i16(y));
  value.set_z(saturate_i32_to_i16(z));

  return true;
}

template <typename I2C, typename Time>
bool MPU6050<I2C, Time>::calibrate_gyro(u16 samples, u16 delay_ms) {
  if (samples == 0)
    return false;

  i32 sx = 0;
  i32 sy = 0;
  i32 sz = 0;

  u16 valid_samples = 0;

  for (u16 i = 0; i < samples; ++i) {
    core::AxesMeasurement gyro;

    if (read_gyro_raw(gyro)) {
      sx += gyro.getAxes().data_x;
      sy += gyro.getAxes().data_y;
      sz += gyro.getAxes().data_z;
      ++valid_samples;
    }

    time_.delay(delay_ms);
  }

  if (valid_samples == 0)
    return false;

  gyro_bias.x = sx / static_cast<i32>(valid_samples);
  gyro_bias.y = sy / static_cast<i32>(valid_samples);
  gyro_bias.z = sz / static_cast<i32>(valid_samples);

  return true;
}

template <typename I2C, typename Time> bool MPU6050<I2C, Time>::init() {
  u8 id = 0;

  if (!read_reg(WHO_AM_I, id))
    return false;

  if ((id != WHO_AM_I_MPU6050) && (id != WHO_AM_I_MPU6500))
    return false;

  // Full device reset
  if (!write_reg(PWR_MGMT_1, PWR_RESET))
    return false;

  time_.delay(100);

  // Wake up, use X gyro PLL as clock source
  if (!write_reg(PWR_MGMT_1, PWR_WAKE_PLL_XGYRO))
    return false;

  time_.delay(10);

  // Enable all accel and gyro axes
  if (!write_reg(PWR_MGMT_2, ALL_AXES_ENABLED))
    return false;

  // Disable FIFO / I2C master features
  if (!write_reg(USER_CTRL, 0x00))
    return false;

  // Reset gyro, accel and temp signal paths
  if (!write_reg(SIGNAL_RESET, 0x07))
    return false;

  time_.delay(100);

  // Digital Low Pass Filter.
  // 0x03 = 44 Hz
  // 0x04 = 20 Hz
  // 0x05 = 10 Hz
  // 0x06 = 5 Hz
  if (!write_reg(MPU_CONFIG, DLPF_20HZ))
    return false;

  // Sample rate = 1 kHz / (9 + 1) = 100 Hz
  if (!write_reg(SMPLRT_DIV, SAMPLE_RATE_100HZ))
    return false;

  // Gyroscope ±250 dps
  if (!write_reg(GYRO_CONFIG, GYRO_FS_250DPS))
    return false;

  // Accelerometer ±2g
  if (!write_reg(ACCEL_CONFIG, ACCEL_FS_2G))
    return false;

  time_.delay(100);

  gyro_bias = {0, 0, 0};

  return true;
}

} // namespace drivers::MPU6050