#pragma once
#include "core-lib/Types.h"

namespace HAL {

///
struct Pin {
  u8 port; // port: 0=A, 1=B, 2=C...
  u8 pin;  // 0..15
};

///
enum class GpioMode : u8 { Input = 0, Output = 1, Alternate = 2, Analog = 3 };

///
enum class OutputMode : u8 {
  PushPull = 0,
  OpenDrain = 1,
};

/// Slew rate
enum class SlewRate : u8 {
  Low = 0,      // ~2Mhz
  Medium = 1,   // ~25Mhz
  High = 2,     // ~50Mhz
  Very_high = 3 // ~100Mhz
};

///
enum class AlternateFunction : u8 {
  AF0 = 0,
  AF1 = 1,
  AF2 = 2,
  AF3 = 3,
  AF4 = 4,
  AF5 = 5,
  AF6 = 6,
  AF7 = 7,
  AF8 = 8,
  AF9 = 9,
  AF10 = 10,
  AF11 = 11,
  AF12 = 12,
  AF13 = 13,
  AF14 = 14,
  AF15 = 15
};

///
enum class GpioPull : u8 { None = 0, Up = 1, Down = 2 };

///
enum class GpioState : u8 { Low = 0, High = 1 };

/**
 *
 */
template <typename Impl> class Gpio {
public:
  /**
   *
   */
  void init(Pin p, GpioMode mode, GpioPull pull) {
    impl().init_impl(p, mode, pull);
  }

  void init(Pin p, GpioMode mode, GpioPull pull, OutputMode output) {
    impl().init_impl(p, mode, pull, output);
  }

  void init(Pin p, GpioMode mode, GpioPull pull, OutputMode output,
            SlewRate slew_rate) {
    impl().init_impl(p, mode, pull, output, slew_rate);
  }

  void set_AF(Pin p, AlternateFunction af) {
    impl().set_AF_impl(p, static_cast<u32>(af));
  }

  void write(Pin p, GpioState s) { impl().write_impl(p, s); }

  GpioState read(Pin p) { return impl().read_impl(p); }

  void toggle(Pin p) {
    // Generic implementation on top of read + write
    const auto cur = read(p);
    write(p, (cur == GpioState::High) ? GpioState::Low : GpioState::High);
  }

private:
  Impl &impl() { return static_cast<Impl &>(*this); }
  const Impl &impl() const { return static_cast<const Impl &>(*this); }
};

} // namespace HAL