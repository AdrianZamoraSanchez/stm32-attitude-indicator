#include "driver/drv_uart_stm32f4.h"
#include "driver/drv_time_stm32f4.h"

namespace drivers::stm32f4 {

u32 UART_STM32F4::get_baudrate_impl() { return baudrate_; }

void UART_STM32F4::set_baudrate_impl(u32 baudrate) {
  if (baudrate == 0U)
    return;

  baudrate_ = baudrate;
  apply_baudrate();
}

void UART_STM32F4::apply_baudrate() {
  // The baud rate generator may only be changed while the USART is disabled
  const bool was_enabled = (USART1->CR1 & USART_CR1_UE) != 0U;
  USART1->CR1 &= ~USART_CR1_UE;

  // Oversampling by 16: BRR = PCLK2 / baud, rounded to nearest
  USART1->BRR = (TimeSTM32F4::pclk2_hz() + (baudrate_ / 2U)) / baudrate_;

  if (was_enabled)
    USART1->CR1 |= USART_CR1_UE;
}

void UART_STM32F4::init_impl() {
  baudrate_ = DEFAULT_BAUDRATE;

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1 clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock (PA9/PA10)
  __DSB();

  USART1->CR1 = 0; // Disabled, 8 data bits, no parity
  USART1->CR2 &= ~USART_CR2_STOP; // 00 = 1 stop bit
  USART1->CR3 = 0;                // No flow control

  apply_baudrate();

  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmitter and receiver
  USART1->CR1 |= USART_CR1_UE;
}

void UART_STM32F4::write_byte_impl(u8 data) { USART1->DR = data; }

void UART_STM32F4::block_write_byte_impl(u8 data) {
  u32 timeout = 0U;

  while ((USART1->SR & USART_SR_TXE) == 0U) {
    if (++timeout >= UART_TX_TIMEOUT)
      return; // Drop the byte rather than hang the main loop
  }

  USART1->DR = data;
}

u8 UART_STM32F4::read_byte_impl() { return static_cast<u8>(USART1->DR); }

u8 UART_STM32F4::block_read_byte_impl() {
  while ((USART1->SR & USART_SR_RXNE) == 0U) {
  }

  return static_cast<u8>(USART1->DR);
}

} // namespace drivers::stm32f4
