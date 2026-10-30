#include "driver/drv_time_stm32f4.h"

extern "C" void SysTick_Handler(void) { HAL::systick_tick(); }

namespace drivers::stm32f4 {

void TimeSTM32F4::systick_init_impl() {
  HSE_PLL();

  SysTick->LOAD = (hclk_hz_ / 1000U) - 1U; // 1 ms tick
  SysTick->VAL = 0;                        // Starting value
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
}

void TimeSTM32F4::HSE_PLL() {
  // Sets HSE as main clock
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY)) // Checks HSE viability
    ;

  // FLASH to 84MHz
  FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
  FLASH->ACR |= FLASH_ACR_ICEN;
  FLASH->ACR |= FLASH_ACR_DCEN;
  FLASH->ACR |= FLASH_ACR_PRFTEN;

  RCC->PLLCFGR = (8 << 0) |              // PLL_M
                 (336 << 6) |            // PLL_N
                 (((4 / 2) - 1) << 16) | // PLL_P
                 RCC_PLLCFGR_PLLSRC_HSE; // HSE

  // Sets PLL as the main clock
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY)) // Checks PLL viability
    ;

  // Prescalers cleaning
  RCC->CFGR &= ~RCC_CFGR_HPRE_DIV1;
  RCC->CFGR &= ~RCC_CFGR_PPRE1_DIV2;
  RCC->CFGR &= ~RCC_CFGR_PPRE2_DIV1;

  // Prescalers setting
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB = 84
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 = 42
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = 84

  // Internal static copy
  sysclk_hz_ = 84000000U;
  hclk_hz_ = 84000000U;
  pclk1_hz_ = 42000000U;
  pclk2_hz_ = 84000000U;

  // Config SYSCLK with PLL
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;
}

} // namespace drivers::stm32f4
