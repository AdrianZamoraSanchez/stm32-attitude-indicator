// Attitude indicator firmware.
//
// Reads an MPU6050 IMU over I2C, estimates roll and pitch with a
// complementary filter and streams them over UART to the PC GUI
// (tools/gui). A 128x32 SSD1306 OLED shows the same values on the board.
//
// Telemetry protocol, one ASCII line per sample:  ATT,<roll>,<pitch>\n
// Angles in degrees, two decimals, vehicle frame.

#include "AppConfig.h"

#include "HAL/HAL_I2C.h"
#include "HAL/HAL_UART.h"
#include "HAL/HAL_gpio.h"
#include "HAL/HAL_time.h"

#include "driver/MPU6050.h"
#include "driver/SSD1306.h"
#include "driver/drv_gpio_stm32f4.h"
#include "driver/drv_i2c_stm32f4.h"
#include "driver/drv_time_stm32f4.h"
#include "driver/drv_uart_stm32f4.h"

#include "core-lib/Attitude.h"
#include "core-lib/StringUtils.h"

namespace {

using GpioDrv = drivers::stm32f4::GpioSTM32F4;
using TimeDrv = drivers::stm32f4::TimeSTM32F4;
using I2CDrv = drivers::stm32f4::I2C_STM32F4;
using UartDrv = drivers::stm32f4::UART_STM32F4;

using Gpio = HAL::Gpio<GpioDrv>;
using Time = HAL::Time<TimeDrv>;
using I2C = HAL::I2C<I2CDrv>;
using Uart = HAL::UART<UartDrv>;

using Imu = drivers::MPU6050::MPU6050<I2C, Time>;
using Display = drivers::SSD1306::SSD1306<I2C>;

using core::attitude::Euler;

/// Unrecoverable init error: blink the status LED fast forever.
[[noreturn]] void fault_blink(Gpio &gpio, Time &time) {
  while (true) {
    gpio.toggle(config::STATUS_LED);
    time.delay(100);
  }
}

void configure_pins(Gpio &gpio) {
  // Status LED
  gpio.init(config::STATUS_LED, HAL::GpioMode::Output, HAL::GpioPull::None);

  // USART1 (AF7)
  gpio.init(config::UART_TX, HAL::GpioMode::Alternate, HAL::GpioPull::Up,
            HAL::OutputMode::PushPull, HAL::SlewRate::High);
  gpio.init(config::UART_RX, HAL::GpioMode::Alternate, HAL::GpioPull::Up,
            HAL::OutputMode::PushPull, HAL::SlewRate::High);
  gpio.set_AF(config::UART_TX, HAL::AlternateFunction::AF7);
  gpio.set_AF(config::UART_RX, HAL::AlternateFunction::AF7);

  // I2C2 (AF4, open drain as required by the bus)
  gpio.init(config::I2C_SCL, HAL::GpioMode::Alternate, HAL::GpioPull::Up,
            HAL::OutputMode::OpenDrain, HAL::SlewRate::High);
  gpio.init(config::I2C_SDA, HAL::GpioMode::Alternate, HAL::GpioPull::Up,
            HAL::OutputMode::OpenDrain, HAL::SlewRate::High);
  gpio.set_AF(config::I2C_SCL, HAL::AlternateFunction::AF4);
  gpio.set_AF(config::I2C_SDA, HAL::AlternateFunction::AF4);
}

/// Send one telemetry line: ATT,<roll>,<pitch>\n
void send_attitude(Uart &uart, const Euler &attitude) {
  char roll[16];
  char pitch[16];

  if (core::f32_to_string(attitude.roll, roll, sizeof(roll), 2) == 0U ||
      core::f32_to_string(attitude.pitch, pitch, sizeof(pitch), 2) == 0U)
    return; // Not representable, skip this sample

  uart.write_string("ATT,");
  uart.write_string(roll);
  uart.write_string(",");
  uart.write_string(pitch);
  uart.write_string("\n");
}

void show_attitude(Display &display, const Euler &attitude, u8 line_height) {
  display.clear();

  display.set_header(0, 0);
  display.print_string("ROLL:  ");
  display.print_f32(attitude.roll);

  display.set_header(0, line_height);
  display.print_string("PITCH: ");
  display.print_f32(attitude.pitch);

  display.draw();
}

} // namespace

int main() {
  GpioDrv gpio_drv;
  TimeDrv time_drv;
  I2CDrv i2c_drv{config::I2C_SPEED};
  UartDrv uart_drv;

  Gpio &gpio = gpio_drv;
  Time &time = time_drv;
  I2C &i2c = i2c_drv;
  Uart &uart = uart_drv;

  // Switch to the 84 MHz PLL and start the 1 ms tick. Must run first:
  // the UART and I2C timings are derived from the resulting bus clocks.
  time.systick_init();

  configure_pins(gpio);

  i2c.init();
  uart.init();
  uart.set_baudrate(config::UART_BAUDRATE);

  screen::Font_5x7 font;
  Display display(i2c, font);

  if (!display.init())
    fault_blink(gpio, time);

  Imu imu(i2c, time);

  if (!imu.init())
    fault_blink(gpio, time);

  // Gyro bias calibration: keep the board still. LED on while it runs.
  gpio.write(config::STATUS_LED, HAL::GpioState::High);
  display.set_header(0, 0);
  display.print_string("CALIBRATING...");
  display.draw();

  if (!imu.calibrate_gyro(config::GYRO_CAL_SAMPLES, 2))
    fault_blink(gpio, time);

  gpio.write(config::STATUS_LED, HAL::GpioState::Low);

  core::attitude::ComplementaryFilter filter(config::FILTER_ALPHA);
  Euler vehicle{0.0f, 0.0f};

  u32 last_imu = HAL::now_ms();
  u32 last_telemetry = last_imu;
  u32 last_display = last_imu;
  u32 last_heartbeat = last_imu;

  // Cooperative scheduler: each task runs when its period has elapsed.
  // `now - last >= period` is safe across the u32 millisecond wrap-around.
  while (true) {
    const u32 now = HAL::now_ms();

    if (now - last_imu >= config::IMU_PERIOD_MS) {
      const f32 dt = static_cast<f32>(now - last_imu) * 0.001f;
      last_imu = now;

      core::AxesMeasurement accel;
      core::AxesMeasurement gyro;

      if (imu.read_accel(accel) && imu.read_gyro(gyro)) {
        const auto a = accel.getAxes();
        const auto g = gyro.getAxes();

        const Euler accel_attitude = core::attitude::from_accel(
            static_cast<f32>(a.data_x), static_cast<f32>(a.data_y),
            static_cast<f32>(a.data_z));

        const Euler sensor = filter.update(
            accel_attitude, Imu::gyro_raw_to_dps(g.data_x),
            Imu::gyro_raw_to_dps(g.data_y), Imu::gyro_raw_to_dps(g.data_z),
            dt);

        vehicle = core::attitude::to_vehicle_frame(sensor, config::SENSOR_MOUNT);
      }
    }

    if (now - last_telemetry >= config::TELEMETRY_PERIOD_MS) {
      last_telemetry = now;
      send_attitude(uart, vehicle);
    }

    if (now - last_display >= config::DISPLAY_PERIOD_MS) {
      last_display = now;
      show_attitude(display, vehicle, font.getHeight());
    }

    if (now - last_heartbeat >= config::HEARTBEAT_PERIOD_MS) {
      last_heartbeat = now;
      gpio.toggle(config::STATUS_LED);
    }
  }
}
