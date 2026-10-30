#pragma once
#include "HAL/HAL_gpio.h"
#include "stm32f407xx.h" // CMSIS device header

namespace drivers::stm32f4 {

constexpr u32 I2C_AF = static_cast<u32>(HAL::AlternateFunction::AF4);

class GpioSTM32F4 : public HAL::Gpio<GpioSTM32F4> {
public:
  /**
   *
   */
  void init_impl(HAL::Pin p, HAL::GpioMode mode, HAL::GpioPull pull);

  /**
   *
   */
  void init_impl(HAL::Pin p, HAL::GpioMode mode, HAL::GpioPull pull,
                 HAL::OutputMode output);

  /**
   *
   */
  void init_impl(HAL::Pin p, HAL::GpioMode mode, HAL::GpioPull pull,
                 HAL::OutputMode output, HAL::SlewRate speed);

  /**
   *
   */
  void set_AF_impl(HAL::Pin p, u32 af);

  /**
   *
   */
  void write_impl(HAL::Pin p, HAL::GpioState s);

  /**
   *
   */
  HAL::GpioState read_impl(HAL::Pin p);

private:
  /**
   *
   */
  static GPIO_TypeDef *port_to_reg(u8 port);

  /**
   *
   */
  static void enable_port_clock(u8 port);
};

} // namespace drivers::stm32f4
