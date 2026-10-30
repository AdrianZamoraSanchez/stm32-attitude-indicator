#pragma once

#include "HAL/HAL_I2C.h"
#include "HAL/HAL_time.h"

#include <array>
#include <vector>

/**
 * Test double for the I2C HAL: emulates one register-based device.
 *
 * Because device drivers are templated on HAL::I2C<Impl> (CRTP), the real
 * driver code runs unchanged against this fake on the host.
 *
 * Model: a write transfer starts with a register address followed by data
 * (auto-increment). A read returns bytes from the current register pointer.
 */
class FakeI2C : public HAL::I2C<FakeI2C> {
public:
  struct RegWrite {
    u8 reg;
    u8 value;
  };

  /// 7-bit address the fake device answers to.
  u8 device_addr = 0x68;

  /// Device register file.
  std::array<u8, 256> regs{};

  /// Every register written by the driver, in order.
  std::vector<RegWrite> writes;

  /// Helper: store a big-endian i16 at reg (high byte) and reg + 1 (low).
  void set_i16(u8 reg, i16 value) {
    const u16 raw = static_cast<u16>(value);
    regs[reg] = static_cast<u8>(raw >> 8);
    regs[static_cast<u8>(reg + 1)] = static_cast<u8>(raw & 0xFF);
  }

  // ---- HAL::I2C implementation ------------------------------------------

  void init_impl() {}

  HAL::I2C_Error start_impl(u8 addr, bool /*read*/, bool /*repeated*/,
                            bool /*clear_addr*/) {
    return (addr == device_addr) ? HAL::I2C_Error::None
                                 : HAL::I2C_Error::AckFailure;
  }

  void stop_impl() {}

  HAL::I2C_Error write_impl(u8 addr, const u8 *data, u16 len) {
    if (addr != device_addr)
      return HAL::I2C_Error::AckFailure;

    if (data == nullptr || len == 0)
      return HAL::I2C_Error::InvalidState;

    pointer_ = data[0];

    for (u16 i = 1; i < len; ++i) {
      regs[pointer_] = data[i];
      writes.push_back({pointer_, data[i]});
      ++pointer_;
    }

    return HAL::I2C_Error::None;
  }

  HAL::I2C_Error write_byte_impl(u8 addr, u8 data) {
    return write_impl(addr, &data, 1);
  }

  HAL::I2C_Error write_raw_impl(const u8 *data, u16 len) {
    if (data == nullptr || len == 0)
      return HAL::I2C_Error::InvalidState;

    pointer_ = data[0]; // Register address phase
    return HAL::I2C_Error::None;
  }

  HAL::I2C_Error read_raw_impl(u8 *data, u16 len) {
    if (data == nullptr || len == 0)
      return HAL::I2C_Error::InvalidState;

    for (u16 i = 0; i < len; ++i) {
      data[i] = regs[pointer_++];
    }

    return HAL::I2C_Error::None;
  }

  HAL::I2C_Error read_impl(u8 addr, u8 *data, u16 len) {
    if (addr != device_addr)
      return HAL::I2C_Error::AckFailure;

    return read_raw_impl(data, len);
  }

private:
  u8 pointer_ = 0; // wraps at 256 like an 8-bit register pointer
};

/// Test double for the time HAL: delays only accumulate virtual time.
class FakeTime : public HAL::Time<FakeTime> {
public:
  u32 elapsed_ms = 0;

  void systick_init_impl() {}
  void delay_impl(u32 ms) { elapsed_ms += ms; }
};
