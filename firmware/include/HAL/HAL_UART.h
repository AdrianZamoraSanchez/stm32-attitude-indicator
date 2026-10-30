#pragma once
#include "core-lib/Types.h"

namespace HAL {

enum class UART_Error {
  None,

  InvalidState,

  TimeoutTxE,
  TimeoutTC,
  TimeoutRxNE,

  Overrun,
  Framing,
  Noise,
  Parity,

  Unknown
};

/**
 * UART interface (CRTP). See HAL_I2C.h for the rationale.
 */
template <typename Impl> class UART {
public:
  /// Configure and enable the peripheral with the default baud rate.
  void init() { impl().init_impl(); }

  u32 get_baudrate() { return impl().get_baudrate_impl(); }

  /// Change the baud rate. Takes effect immediately.
  void set_baudrate(u32 baudrate) { impl().set_baudrate_impl(baudrate); }

  /// Write one byte without waiting for the transmit register to be empty.
  void write_byte(u8 data) { impl().write_byte_impl(data); }

  /// Wait (with timeout) until the transmit register is empty, then write.
  void block_write_byte(u8 data) { impl().block_write_byte_impl(data); }

  /// Write a null-terminated string, byte by byte.
  void write_string(const char *str) {
    if (str == nullptr)
      return;

    while (*str != '\0') {
      block_write_byte(static_cast<u8>(*str));
      ++str;
    }
  }

  /// Read the data register without waiting.
  u8 read_byte() { return impl().read_byte_impl(); }

  /// Block until a byte is received.
  u8 block_read_byte() { return impl().block_read_byte_impl(); }

private:
  Impl &impl() { return static_cast<Impl &>(*this); }
};

} // namespace HAL
