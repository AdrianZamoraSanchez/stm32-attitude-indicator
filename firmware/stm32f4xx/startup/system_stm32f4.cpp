#include "stm32f407xx.h"

// Called by Reset_Handler before .data/.bss are initialized and before any
// static constructor runs, so it must not touch global variables.
// The system clock is configured later by TimeSTM32F4::systick_init().
extern "C" void SystemInit(void) {
  // Enable full access to the FPU (CP10 and CP11). The firmware is built with
  // -mfloat-abi=hard, so this must happen before the first floating point
  // instruction, including any in static constructors. Doing it in main()
  // is too late.
  SCB->CPACR |= (0xFU << 20);
  __DSB();
  __ISB();
}
