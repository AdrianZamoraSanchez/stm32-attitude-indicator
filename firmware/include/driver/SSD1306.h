#pragma once

#include "HAL/HAL_I2C.h"
#include "screen/Font.h"

namespace drivers::SSD1306 {

constexpr u8 SSD1306_ADDR = 0x3C;

constexpr u8 SSD1306_WIDTH = 128;
constexpr u8 SSD1306_HEIGHT = 32;
constexpr u16 SSD1306_BUFF_SZ = (SSD1306_WIDTH * SSD1306_HEIGHT) / 8; // 512

// First byte of every I2C transfer: tells the controller what follows
constexpr u8 CONTROL_COMMANDS = 0x00;
constexpr u8 CONTROL_DATA = 0x40;

/**
 * Driver for a 128x32 SSD1306 OLED over I2C, with a RAM framebuffer.
 * Drawing functions only touch the framebuffer; call draw() to send it.
 */
template <typename I2C> class SSD1306 {
public:
  SSD1306(I2C &i2c, screen::Font &font);

  // Not copyable: `buff` points into this object's own frame_
  SSD1306(const SSD1306 &) = delete;
  SSD1306 &operator=(const SSD1306 &) = delete;

  /// Send the init sequence and show an empty screen.
  bool init();

  /// Send the whole framebuffer (513 bytes; ~46 ms at 100 kHz).
  bool draw();

  /// Clear the framebuffer and move the cursor home (does not draw).
  void clear();

  void setPixel(u8 x, u8 y, bool on);
  void print_char(char ch);
  void print_string(const char *str);
  void print_u16(u16 value);
  void print_u8(u8 value);
  void print_i16(i16 value);
  void print_i8(i8 value);
  void print_f32(f32 value);

  /// Move the text cursor (pixels).
  void set_header(u16 x, u16 y);

private:
  I2C &i2c_;

  u16 x_coord;
  u16 y_coord;

  screen::Font &font;

  // Data control byte followed by the pixels, so draw() sends it as is
  u8 frame_[1 + SSD1306_BUFF_SZ];
  u8 *const buff = &frame_[1];
};

template <typename I2C> void SSD1306<I2C>::set_header(u16 x, u16 y) {
  x_coord = x;
  y_coord = y;
}

template <typename I2C>
SSD1306<I2C>::SSD1306(I2C &i2c, screen::Font &font)
    : i2c_(i2c), x_coord(0), y_coord(0), font(font), frame_{} {
  frame_[0] = CONTROL_DATA;
}

template <typename I2C> bool SSD1306<I2C>::init() {

  const u8 init_seq[] = {
      CONTROL_COMMANDS,

      0xAE, // Display OFF

      0xD5, 0x80, // Clock divide ratio / oscillator freq

      0xA8, 0x1F, // Multiplex ratio (128x32)

      0xD3, 0x00, // Display offset
      0x40,       // Start line = 0

      0x8D, 0x14, // Charge pump

      0x20, 0x00, // Memory addressing mode = horizontal

      0xA1, // Segment remap
      0xC8, // COM scan direction

      0xDA, 0x02, // COM pins hardware config

      0x81, 0x8F, // Contrast

      0xD9, 0xF1, // Pre-charge period

      0xDB, 0x40, // VCOMH deselect level

      0xA4, // Resume RAM display
      0xA6, // Normal display (no invert)

      0xAF // Display ON
  };

  if (i2c_.write(SSD1306_ADDR, init_seq, sizeof(init_seq)) !=
      HAL::I2C_Error::None)
    return false;

  clear();

  return draw();
}

template <typename I2C> bool SSD1306<I2C>::draw() {

  // Address window: columns 0..127, pages 0..3
  const u8 draw_preparation[] = {CONTROL_COMMANDS, 0x21, 0x00, 0x7F,
                                 0x22,             0x00, 0x03};

  if (i2c_.write(SSD1306_ADDR, draw_preparation, sizeof(draw_preparation)) !=
      HAL::I2C_Error::None)
    return false;

  return i2c_.write(SSD1306_ADDR, frame_, sizeof(frame_)) ==
         HAL::I2C_Error::None;
}

template <typename I2C> void SSD1306<I2C>::clear() {
  for (u16 i = 0; i < SSD1306_BUFF_SZ; ++i)
    buff[i] = 0;

  x_coord = 0;
  y_coord = 0;
}

template <typename I2C> void SSD1306<I2C>::print_u16(u16 value) {
  char buffer[6]; // 5 digits + '\0'
  int i = 0;

  // Case 0
  if (value == 0) {
    print_char('0');
    return;
  }

  // Inverse order generation
  while (value > 0) {
    buffer[i++] = '0' + (value % 10);
    value /= 10;
  }

  // Right order print
  for (int j = i - 1; j >= 0; --j) {
    print_char(buffer[j]);
  }
}

template <typename I2C> void SSD1306<I2C>::print_u8(u8 value) {
  print_u16(static_cast<u16>(value));
}

template <typename I2C> void SSD1306<I2C>::print_i8(i8 value) {
  print_i16(static_cast<i16>(value));
}

template <typename I2C> void SSD1306<I2C>::print_i16(i16 value) {
  if (value < 0) {
    print_char('-');

    // Using i32 to prevent that -(-32768) overflows i16
    i32 v = static_cast<i32>(value);
    u16 abs_value = static_cast<u16>(-v);

    print_u16(abs_value);
    return;
  }

  print_u16(static_cast<u16>(value));
}

template <typename I2C> void SSD1306<I2C>::print_f32(f32 value) {
  // Handle negative values
  if (value < 0.0f) {
    print_char('-');
    value = -value;
  }

  // Integer part
  u16 integer_part = static_cast<u16>(value);

  // Fractional part, 2 decimals
  f32 fractional = value - static_cast<f32>(integer_part);
  u16 decimals = static_cast<u16>((fractional * 100.0f) + 0.5f); // rounding

  // Rounding overflow: 1.999 -> 2.00
  if (decimals >= 100) {
    integer_part += 1;
    decimals -= 100;
  }

  print_u16(integer_part);
  print_char('.');

  // Force two digits
  if (decimals < 10)
    print_char('0');

  print_u16(decimals);
}

template <typename I2C> void SSD1306<I2C>::print_string(const char *str) {
  if (!str)
    return;

  while (*str != '\0') {
    print_char(*str);
    ++str;
  }
}

template <typename I2C> void SSD1306<I2C>::setPixel(u8 x, u8 y, bool on) {
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    return;

  // Horizontal addressing: 8-pixel tall pages, one byte per column
  u16 index = (y / 8) * SSD1306_WIDTH + x;
  u8 bit = y % 8;

  if (on)
    buff[index] |= (1U << bit);
  else
    buff[index] &= ~(1U << bit);
}

template <typename I2C> void SSD1306<I2C>::print_char(char ch) {
  auto cols = font.getData(ch);

  for (u8 col = 0; col < font.getWidth(); ++col) {
    for (u8 row = 0; row < font.getHeight(); ++row) {
      bool on = cols[col] & (1 << row);
      setPixel(x_coord + col, y_coord + row, on);
    }
  }

  // Advance the cursor (1 px spacing between characters)
  x_coord += font.getWidth() + 1;

  if (x_coord > SSD1306_WIDTH - font.getWidth()) {
    x_coord = 0;
    y_coord += font.getHeight();
  }

  if (y_coord > SSD1306_HEIGHT - font.getHeight()) {
    y_coord = 0;
  }
}

} // namespace drivers::SSD1306
