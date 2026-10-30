#pragma once
#include "core-lib/Types.h"

namespace HAL {

enum class I2C_Error {
  None,

  Busy,
  InvalidState,

  TimeoutStart,
  TimeoutAddr,
  TimeoutTxE,
  TimeoutRxNE,
  TimeoutBTF,

  AckFailure,
  BusError,
  ArbitrationLost,
  Overrun
};

/**
 * I2C master interface (CRTP).
 *
 * Device drivers are templated on this interface, so they never depend on a
 * specific MCU. Calls are resolved at compile time: no vtable, no overhead.
 * `Impl` must provide the `*_impl` functions used below.
 */
template <typename Impl> class I2C {
public:
  /// Configure and enable the peripheral.
  void init() { impl().init_impl(); }

  /**
   * Generate a START (or repeated START) and send the 7-bit address.
   * @param read        true for a read transfer, false for a write.
   * @param repeated    true when issuing a repeated START inside a transfer.
   * @param clear_addr  clear the ADDR flag once the address is acknowledged.
   */
  I2C_Error start(u8 addr, bool read, bool repeated = false,
                  bool clear_addr = true) {
    return impl().start_impl(addr, read, repeated, clear_addr);
  }

  /// Generate a STOP condition.
  void stop() { impl().stop_impl(); }

  /// Complete write transfer: START, address, `len` bytes, STOP.
  I2C_Error write(u8 addr, const u8 *data, u16 len) {
    return impl().write_impl(addr, data, len);
  }

  /// Complete single-byte write transfer.
  I2C_Error write_byte(u8 addr, u8 data) {
    return impl().write_byte_impl(addr, data);
  }

  /// Send bytes inside a transfer already opened with start().
  I2C_Error write_raw(const u8 *data, u16 len) {
    return impl().write_raw_impl(data, len);
  }

  /// Receive bytes inside a transfer already opened with start().
  /// Generates the STOP condition after the last byte.
  I2C_Error read_raw(u8 *data, u16 len) {
    return impl().read_raw_impl(data, len);
  }

  /// Complete read transfer: START, address, `len` bytes, STOP.
  I2C_Error read(u8 addr, u8 *data, u16 len) {
    return impl().read_impl(addr, data, len);
  }

private:
  Impl &impl() { return static_cast<Impl &>(*this); }
};

} // namespace HAL
