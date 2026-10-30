#pragma once
#include "core-lib/Types.h"
#include <atomic>

namespace HAL {

/// Milliseconds since boot, incremented by the SysTick interrupt.
/// A u32 overflows every ~49 days; elapsed-time checks written as
/// `now - start >= period` stay correct across the overflow.
inline std::atomic<u32> TIME{0};

/// Current time in milliseconds.
inline u32 now_ms() { return TIME.load(std::memory_order_relaxed); }

/// Called from the SysTick interrupt handler.
inline void systick_tick() { TIME.fetch_add(1, std::memory_order_relaxed); }

/**
 * Time interface (CRTP). See HAL_I2C.h for the rationale.
 */
template <typename Impl> class Time {
public:
  /// Configure the system clock and start the 1 ms SysTick.
  void systick_init() { impl().systick_init_impl(); }

  /// Busy-wait for `delay` milliseconds.
  void delay(u32 delay) { impl().delay_impl(delay); }

private:
  Impl &impl() { return static_cast<Impl &>(*this); }
};

} // namespace HAL
