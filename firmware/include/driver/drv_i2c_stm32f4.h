#pragma once

#include "HAL/HAL_I2C.h"
#include "stm32f407xx.h" // CMSIS device header

namespace drivers::stm32f4 {

/// Supported bus speeds.
enum class I2C_Speed : u8 {
  Standard50kHz,
  Standard100kHz,
  Fast400kHz,
};

/**
 * Register-level I2C2 master driver for the STM32F4 (polling, no ST HAL).
 * Pins: PB10 = SCL, PB11 = SDA (AF4, open drain), configured by the app.
 */
class I2C_STM32F4 : public HAL::I2C<I2C_STM32F4> {
public:
  /// Busy-wait iterations before a flag wait is reported as a timeout.
  static constexpr u32 TIMEOUT = 100000;

  explicit I2C_STM32F4(I2C_Speed speed = I2C_Speed::Standard100kHz)
      : speed_(speed) {}

  /// Enable the clock, reset the peripheral and program the bus timing.
  /// Call after the system clock is configured (uses PCLK1).
  void init_impl();

  /// START + address + data + STOP.
  HAL::I2C_Error write_impl(u8 addr, const u8 *data, u16 len);

  /// START + address + one byte + STOP.
  HAL::I2C_Error write_byte_impl(u8 addr, u8 data);

  /// Send data inside an open transfer (no START/STOP).
  HAL::I2C_Error write_raw_impl(const u8 *data, u16 len);

  /// START + address + read + STOP.
  HAL::I2C_Error read_impl(u8 addr, u8 *data, u16 len);

  /// Read inside an open transfer; generates STOP after the last byte.
  HAL::I2C_Error read_raw_impl(u8 *data, u16 len);

  /// Generate START (or repeated START) and send the address.
  HAL::I2C_Error start_impl(u8 addr, bool read, bool repeated = false,
                            bool clear_addr = true);

  /// Generate STOP.
  void stop_impl();

  void set_pos(bool enable);
  void set_ack(bool enable);

  /// Restore ACK on, POS off and clear a pending acknowledge failure.
  void reset_state();

private:
  /// Spin until `mask` is set in `reg`, or return `err` after `timeout` loops.
  HAL::I2C_Error wait_flag_set(volatile u32 &reg, u32 mask, u32 timeout,
                               HAL::I2C_Error err);

  I2C_Speed speed_;
};

} // namespace drivers::stm32f4
