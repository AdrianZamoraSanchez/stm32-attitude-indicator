#pragma once

#include "HAL/HAL_gpio.h"
#include "core-lib/Attitude.h"
#include "driver/drv_i2c_stm32f4.h"

// Application configuration: pins, rates and tuning in one place.

namespace config {

// ---- Pins ----------------------------------------------------------------
constexpr HAL::Pin STATUS_LED{0, 1}; // PA1
constexpr HAL::Pin UART_TX{0, 9};    // PA9  -> USB-UART RX
constexpr HAL::Pin UART_RX{0, 10};   // PA10 <- USB-UART TX
constexpr HAL::Pin I2C_SCL{1, 10};   // PB10 (I2C2)
constexpr HAL::Pin I2C_SDA{1, 11};   // PB11 (I2C2)

// ---- Buses ---------------------------------------------------------------
constexpr u32 UART_BAUDRATE = 115200U;

// 400 kHz makes the OLED refresh ~4x faster, but needs strong enough
// pull-ups on SDA/SCL. Fall back to Standard50kHz on a long or noisy bus.
constexpr auto I2C_SPEED = drivers::stm32f4::I2C_Speed::Standard100kHz;

// ---- Scheduling (main loop periods) --------------------------------------
constexpr u32 IMU_PERIOD_MS = 10U;       // 100 Hz, matches the MPU6050 ODR
constexpr u32 TELEMETRY_PERIOD_MS = 20U; // 50 Hz to the PC
constexpr u32 DISPLAY_PERIOD_MS = 250U;  // 4 Hz: a full OLED frame is slow
constexpr u32 HEARTBEAT_PERIOD_MS = 500U;

// ---- Attitude estimation -------------------------------------------------
// Gyro bias calibration at boot. The board must stay still meanwhile.
constexpr u16 GYRO_CAL_SAMPLES = 500U; // x 2 ms = ~1 s

// Complementary filter weight. 0.0 = accelerometer only, 1.0 = gyro only.
constexpr f32 FILTER_ALPHA = 0.98f;

// The MPU6050 board is mounted rotated with respect to the vehicle axes.
// Sensor roll/pitch are swapped and inverted to get vehicle roll/pitch.
// Determined empirically for the current mounting (see README).
constexpr core::attitude::MountCompensation SENSOR_MOUNT{
    true,  // swap_roll_pitch
    -1.0f, // roll_sign
    -1.0f, // pitch_sign
};

} // namespace config
