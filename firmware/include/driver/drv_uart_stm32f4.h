#pragma once

#include "HAL/HAL_UART.h"
#include "stm32f407xx.h" // CMSIS device header

namespace drivers::stm32f4 {

/// Busy-wait iterations before a transmit is abandoned. Generous on purpose:
/// one byte at 115200 baud takes ~87 us (~7300 CPU cycles at 84 MHz).
constexpr u32 UART_TX_TIMEOUT = 100000U;

/**
 * Register-level USART1 driver for the STM32F4 (polling, 8N1).
 * Pins: PA9 = TX, PA10 = RX (AF7), configured by the app.
 */
class UART_STM32F4 : public HAL::UART<UART_STM32F4> {
public:
  static constexpr u32 DEFAULT_BAUDRATE = 115200U;

  /// Enable the clock and configure 8N1 at DEFAULT_BAUDRATE.
  /// Call after the system clock is configured (uses PCLK2).
  void init_impl();

  u32 get_baudrate_impl();

  /// Reprogram the baud rate generator.
  void set_baudrate_impl(u32 baudrate);

  void write_byte_impl(u8 data);

  /// Wait for TXE (with timeout) and write. On timeout the byte is dropped
  /// instead of hanging the main loop.
  void block_write_byte_impl(u8 data);

  u8 read_byte_impl();

  /// Block until RXNE is set (no timeout).
  u8 block_read_byte_impl();

private:
  void apply_baudrate();

  u32 baudrate_ = DEFAULT_BAUDRATE;
};

} // namespace drivers::stm32f4
