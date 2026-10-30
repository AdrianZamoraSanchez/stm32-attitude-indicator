#include "driver/drv_gpio_stm32f4.h"

namespace drivers::stm32f4 {

GPIO_TypeDef *GpioSTM32F4::port_to_reg(u8 port) {
  switch (port) {
  case 0:
    return GPIOA;
  case 1:
    return GPIOB;
  case 2:
    return GPIOC;
  case 3:
    return GPIOD;
  case 4:
    return GPIOE;
  default:
    return nullptr;
  }
}

void GpioSTM32F4::enable_port_clock(u8 port) {
  // AHB1ENR bits: GPIOAEN=0, GPIOBEN=1, etc
  if (port <= 8) {
    RCC->AHB1ENR |= (1U << port);
    // Barrier to ensure clock is active before changing registers
    // GPIO
    __DSB();
  }
}

void GpioSTM32F4::set_AF_impl(HAL::Pin p, u32 af) {
  if (p.pin > 15)
    return;

  GPIO_TypeDef *gpio = port_to_reg(p.port);
  if (!gpio)
    return;

  const u32 index = (p.pin < 8) ? 0U : 1U;
  const u32 shift = (p.pin % 8U) * 4U;

  gpio->AFR[index] &= ~(0xFU << shift);
  gpio->AFR[index] |= (af << shift);
}

void GpioSTM32F4::init_impl(HAL::Pin p, HAL::GpioMode mode,
                            HAL::GpioPull pull) {
  if (p.pin > 15)
    return;

  GPIO_TypeDef *gpio = port_to_reg(p.port);
  if (!gpio)
    return;

  enable_port_clock(p.port);

  const u32 shift = static_cast<u32>(p.pin) * 2U;

  // MODER (2 bits each pin)
  gpio->MODER &= ~(0x3U << shift);
  gpio->MODER |= (static_cast<u32>(mode) << shift);

  // PUPDR (2 bits each pin)
  gpio->PUPDR &= ~(0x3U << shift);
  gpio->PUPDR |= (static_cast<u32>(pull) << shift);
}

void GpioSTM32F4::init_impl(HAL::Pin p, HAL::GpioMode mode, HAL::GpioPull pull,
                            HAL::OutputMode output) {
  if (p.pin > 15)
    return;

  GPIO_TypeDef *gpio = port_to_reg(p.port);
  if (!gpio)
    return;

  enable_port_clock(p.port);

  const u32 shift = static_cast<u32>(p.pin) * 2U;

  // MODER (2 bits each pin)
  gpio->MODER &= ~(0x3U << shift);
  gpio->MODER |= (static_cast<u32>(mode) << shift);

  // PUPDR (2 bits each pin)
  gpio->PUPDR &= ~(0x3U << shift);
  gpio->PUPDR |= (static_cast<u32>(pull) << shift);

  // Output mode (0 = push-pull / 1 = open-drain)
  gpio->OTYPER &= ~(1u << p.pin);
  gpio->OTYPER |= (static_cast<uint32_t>(output) << p.pin);
}

void GpioSTM32F4::init_impl(HAL::Pin p, HAL::GpioMode mode, HAL::GpioPull pull,
                            HAL::OutputMode output, HAL::SlewRate slew_rate) {
  if (p.pin > 15)
    return;

  GPIO_TypeDef *gpio = port_to_reg(p.port);
  if (!gpio)
    return;

  enable_port_clock(p.port);

  const u32 shift = static_cast<u32>(p.pin) * 2U;

  // MODER (2 bits each pin)
  gpio->MODER &= ~(0x3U << shift);
  gpio->MODER |= (static_cast<u32>(mode) << shift);

  // PUPDR (2 bits each pin)
  gpio->PUPDR &= ~(0x3U << shift);
  gpio->PUPDR |= (static_cast<u32>(pull) << shift);

  // Output mode (0 = push-pull / 1 = open-drain)
  gpio->OTYPER &= ~(1u << p.pin);
  gpio->OTYPER |= (static_cast<uint32_t>(output) << p.pin);

  // Slew rate
  gpio->OSPEEDR &= ~(3u << shift);
  gpio->OSPEEDR |= (static_cast<uint32_t>(slew_rate) << shift);
}

void GpioSTM32F4::write_impl(HAL::Pin p, HAL::GpioState s) {
  if (p.pin > 15)
    return;

  GPIO_TypeDef *gpio = port_to_reg(p.port);
  if (!gpio)
    return;

  if (s == HAL::GpioState::High) {
    gpio->BSRR = (1U << p.pin); // set
  } else {
    gpio->BSRR = (1U << (p.pin + 16U)); // reset
  }
}

HAL::GpioState GpioSTM32F4::read_impl(HAL::Pin p) {
  if (p.pin > 15)
    return HAL::GpioState::Low;

  GPIO_TypeDef *gpio = port_to_reg(p.port);
  if (!gpio)
    return HAL::GpioState::Low;

  return (gpio->IDR & (1U << p.pin)) ? HAL::GpioState::High
                                     : HAL::GpioState::Low;
}

} // namespace drivers::stm32f4
