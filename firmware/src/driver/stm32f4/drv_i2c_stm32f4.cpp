#include "driver/drv_i2c_stm32f4.h"
#include "HAL/HAL_I2C.h"
#include "driver/drv_time_stm32f4.h"

namespace drivers::stm32f4 {

HAL::I2C_Error I2C_STM32F4::wait_flag_set(volatile u32 &reg, u32 mask,
                                          u32 timeout, HAL::I2C_Error err) {
  while ((reg & mask) == 0U) {
    if (--timeout == 0U)
      return err;
  }
  return HAL::I2C_Error::None;
}

void I2C_STM32F4::stop_impl() { I2C2->CR1 |= I2C_CR1_STOP; }

void I2C_STM32F4::init_impl() {
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  __DSB();

  // Disable peripheral
  I2C2->CR1 &= ~I2C_CR1_PE;

  // Software reset I2C peripheral (recovers from a stuck BUSY state)
  I2C2->CR1 |= I2C_CR1_SWRST;
  for (u32 i = 0U; i < 1000U; ++i) {
    __NOP();
  }
  I2C2->CR1 &= ~I2C_CR1_SWRST;

  // Bus timing, derived from the real APB1 clock (42 MHz after HSE_PLL())
  const u32 pclk1_hz = TimeSTM32F4::pclk1_hz();
  const u32 pclk1_mhz = pclk1_hz / 1000000U;

  I2C2->CR2 = pclk1_mhz; // FREQ field

  switch (speed_) {
  case I2C_Speed::Standard50kHz:
    I2C2->CCR = pclk1_hz / (2U * 50000U); // Thigh = Tlow = CCR * Tpclk1
    I2C2->TRISE = pclk1_mhz + 1U;         // 1000 ns max rise time
    break;

  case I2C_Speed::Standard100kHz:
    I2C2->CCR = pclk1_hz / (2U * 100000U);
    I2C2->TRISE = pclk1_mhz + 1U;
    break;

  case I2C_Speed::Fast400kHz:
    // Fast mode, duty 2:1 -> Tlow = 2 * Thigh, period = 3 * CCR * Tpclk1
    I2C2->CCR = I2C_CCR_FS | (pclk1_hz / (3U * 400000U));
    I2C2->TRISE = ((pclk1_mhz * 300U) / 1000U) + 1U; // 300 ns max rise time
    break;
  }

  // Clear/restore sane state
  I2C2->CR1 |= I2C_CR1_ACK;

  // Enable peripheral
  I2C2->CR1 |= I2C_CR1_PE;
}

HAL::I2C_Error I2C_STM32F4::start_impl(u8 addr, bool read, bool repeated,
                                       bool clear_addr) {
  u32 timeout = TIMEOUT;

  // Restore sane state before every transaction
  I2C2->CR1 |= I2C_CR1_ACK;
  I2C2->CR1 &= ~I2C_CR1_POS;

  // Clear stale AF
  I2C2->SR1 &= ~I2C_SR1_AF;

  // Clear stale AF before new address phase
  if (I2C2->SR1 & I2C_SR1_AF)
    I2C2->SR1 &= ~I2C_SR1_AF;

  // Repeated start
  if (!repeated) {
    while (I2C2->SR2 & I2C_SR2_BUSY) {
      if (--timeout == 0U)
        return HAL::I2C_Error::Busy;
    }
  }

  I2C2->CR1 |= I2C_CR1_START;

  auto err = wait_flag_set(I2C2->SR1, I2C_SR1_SB, TIMEOUT,
                           HAL::I2C_Error::TimeoutStart);
  if (err != HAL::I2C_Error::None)
    return err;

  I2C2->DR = (addr << 1) | (read ? 1U : 0U);

  timeout = TIMEOUT;
  while (!(I2C2->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) {
    if (--timeout == 0U)
      return HAL::I2C_Error::TimeoutAddr;
  }

  if (I2C2->SR1 & I2C_SR1_AF) {
    I2C2->SR1 &= ~I2C_SR1_AF;
    return HAL::I2C_Error::AckFailure;
  }

  if (clear_addr) {
    (void)I2C2->SR1;
    (void)I2C2->SR2;
  }

  return HAL::I2C_Error::None;
}

HAL::I2C_Error I2C_STM32F4::write_impl(u8 addr, const u8 *data, u16 len) {
  if (data == nullptr || len == 0)
    return HAL::I2C_Error::InvalidState;

  auto err = start(addr, false);
  if (err != HAL::I2C_Error::None)
    return err;

  for (u16 i = 0; i < len; ++i) {

    err = wait_flag_set(I2C2->SR1, I2C_SR1_TXE, TIMEOUT,
                        HAL::I2C_Error::TimeoutTxE);
    if (err != HAL::I2C_Error::None) {
      stop_impl();
      return err;
    }

    I2C2->DR = data[i];

    err = wait_flag_set(I2C2->SR1, I2C_SR1_BTF, TIMEOUT,
                        HAL::I2C_Error::TimeoutBTF);
    if (err != HAL::I2C_Error::None) {
      stop_impl();
      return err;
    }
  }

  stop_impl();

  // Flush state
  volatile u32 tmp;

  tmp = I2C2->SR1;
  tmp = I2C2->SR2;

  (void)tmp;

  // Restore sane state
  I2C2->CR1 |= I2C_CR1_ACK;
  I2C2->CR1 &= ~I2C_CR1_POS;

  // Clear AF
  I2C2->SR1 &= ~I2C_SR1_AF;

  return HAL::I2C_Error::None;
}

HAL::I2C_Error I2C_STM32F4::write_byte_impl(u8 addr, u8 data) {

  auto err = start(addr, false);
  if (err != HAL::I2C_Error::None)
    return err;

  err = wait_flag_set(I2C2->SR1, I2C_SR1_TXE, TIMEOUT,
                      HAL::I2C_Error::TimeoutTxE);
  if (err != HAL::I2C_Error::None) {
    stop_impl();
    return err;
  }

  I2C2->DR = data;

  err = wait_flag_set(I2C2->SR1, I2C_SR1_BTF, TIMEOUT,
                      HAL::I2C_Error::TimeoutBTF);
  if (err != HAL::I2C_Error::None) {
    stop_impl();
    return err;
  }

  stop_impl();

  // Flush state
  volatile u32 tmp;

  tmp = I2C2->SR1;
  tmp = I2C2->SR2;

  (void)tmp;

  // Restore sane state
  I2C2->CR1 |= I2C_CR1_ACK;
  I2C2->CR1 &= ~I2C_CR1_POS;

  // Clear AF
  I2C2->SR1 &= ~I2C_SR1_AF;

  return HAL::I2C_Error::None;
}

HAL::I2C_Error I2C_STM32F4::write_raw_impl(const u8 *data, u16 len) {

  if (data == nullptr || len == 0)
    return HAL::I2C_Error::InvalidState;

  for (u16 i = 0; i < len; ++i) {

    auto err = wait_flag_set(I2C2->SR1, I2C_SR1_TXE, TIMEOUT,
                             HAL::I2C_Error::TimeoutTxE);

    if (err != HAL::I2C_Error::None)
      return err;

    I2C2->DR = data[i];

    // LAST BYTE -> wait BTF
    if (i == (len - 1)) {

      err = wait_flag_set(I2C2->SR1, I2C_SR1_BTF, TIMEOUT,
                          HAL::I2C_Error::TimeoutBTF);

      if (err != HAL::I2C_Error::None)
        return err;
    }
  }

  return HAL::I2C_Error::None;
}

HAL::I2C_Error I2C_STM32F4::read_impl(u8 addr, u8 *data, u16 len) {
  if (data == nullptr || len == 0)
    return HAL::I2C_Error::InvalidState;

  auto err = start(addr, true);
  if (err != HAL::I2C_Error::None)
    return err;

  if (len == 1)
    I2C2->CR1 &= ~I2C_CR1_ACK;
  else
    I2C2->CR1 |= I2C_CR1_ACK;

  for (u16 i = 0; i < len; ++i) {

    if (i == (len - 1))
      I2C2->CR1 &= ~I2C_CR1_ACK;

    err = wait_flag_set(I2C2->SR1, I2C_SR1_RXNE, TIMEOUT,
                        HAL::I2C_Error::TimeoutRxNE);
    if (err != HAL::I2C_Error::None) {
      stop_impl();
      I2C2->CR1 |= I2C_CR1_ACK;
      return err;
    }

    data[i] = I2C2->DR;
  }

  stop_impl();
  I2C2->CR1 |= I2C_CR1_ACK;

  return HAL::I2C_Error::None;
}

HAL::I2C_Error I2C_STM32F4::read_raw_impl(u8 *data, u16 len) {

  if (data == nullptr || len == 0)
    return HAL::I2C_Error::InvalidState;

  // =========================
  // SPECIAL CASE: 2 BYTES
  // =========================
  if (len == 2) {

    // Clear ADDR NOW
    (void)I2C2->SR1;
    (void)I2C2->SR2;

    // ACK = 0
    I2C2->CR1 &= ~I2C_CR1_ACK;

    // Wait until both bytes received
    auto err = wait_flag_set(I2C2->SR1, I2C_SR1_BTF, TIMEOUT,
                             HAL::I2C_Error::TimeoutBTF);

    if (err != HAL::I2C_Error::None) {

      stop_impl();

      I2C2->CR1 |= I2C_CR1_ACK;

      return err;
    }

    // STOP before reading
    stop_impl();

    u8 b1 = I2C2->DR;
    u8 b2 = I2C2->DR;

    data[0] = b2;
    data[1] = b1;

    // Restore ACK
    I2C2->CR1 |= I2C_CR1_ACK;

    return HAL::I2C_Error::None;
  }

  // =========================
  // GENERIC MULTIBYTE
  // =========================

  I2C2->CR1 |= I2C_CR1_ACK;

  for (u16 i = 0; i < len; ++i) {

    if (i == (len - 1)) {

      I2C2->CR1 &= ~I2C_CR1_ACK;

      stop_impl();
    }

    auto err = wait_flag_set(I2C2->SR1, I2C_SR1_RXNE, TIMEOUT,
                             HAL::I2C_Error::TimeoutRxNE);

    if (err != HAL::I2C_Error::None) {

      I2C2->CR1 |= I2C_CR1_ACK;

      return err;
    }

    data[i] = I2C2->DR;
  }

  I2C2->CR1 |= I2C_CR1_ACK;

  return HAL::I2C_Error::None;
}

void I2C_STM32F4::set_pos(bool enable) {

  if (enable)
    I2C2->CR1 |= I2C_CR1_POS;
  else
    I2C2->CR1 &= ~I2C_CR1_POS;
}

void I2C_STM32F4::set_ack(bool enable) {

  if (enable)
    I2C2->CR1 |= I2C_CR1_ACK;
  else
    I2C2->CR1 &= ~I2C_CR1_ACK;
}

void I2C_STM32F4::reset_state() {

  // ACK enabled
  I2C2->CR1 |= I2C_CR1_ACK;

  // POS disabled
  I2C2->CR1 &= ~I2C_CR1_POS;

  // Clear AF
  I2C2->SR1 &= ~I2C_SR1_AF;
}

} // namespace drivers::stm32f4