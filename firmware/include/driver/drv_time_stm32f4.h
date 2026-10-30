#pragma once
#include "HAL/HAL_time.h"
#include "stm32f407xx.h" // CMSIS device header

namespace drivers::stm32f4 {

class TimeSTM32F4 : public HAL::Time<TimeSTM32F4> {
public:
  /**
   *
   */
  void systick_init_impl();

  inline void delay_impl(u32 delay) {
    u32 initial_time = HAL::now_ms();

    while ((u32)(HAL::now_ms() - initial_time) < delay) {
      __NOP(); // DEBUG
      //__WFI(); // RELEASE
    }
  }

  /**
   *
   */
  void HSE_PLL();

  static u32 sysclk_hz() { return sysclk_hz_; }
  static u32 hclk_hz() { return hclk_hz_; }
  static u32 pclk1_hz() { return pclk1_hz_; }
  static u32 pclk2_hz() { return pclk2_hz_; }

private:
  static inline u32 sysclk_hz_ = 16000000U; // HSI reset default
  static inline u32 hclk_hz_ = 16000000U;
  static inline u32 pclk1_hz_ = 16000000U;
  static inline u32 pclk2_hz_ = 16000000U;
};
} // namespace drivers::stm32f4
