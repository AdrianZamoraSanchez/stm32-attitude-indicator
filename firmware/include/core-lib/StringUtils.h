#pragma once
#include "core-lib/Types.h"

// Allocation-free number formatting (no printf on the target).

namespace core {

/**
 * Write `value` in decimal into `buffer`, null-terminated.
 * @return number of characters written (without the terminator),
 *         or 0 if the buffer is too small.
 */
inline u32 u32_to_string(u32 value, char *buffer, u32 buffer_size) {
  if (buffer == nullptr || buffer_size < 2U)
    return 0U;

  char reversed[10]; // u32 max is 10 digits
  u32 digits = 0U;

  do {
    reversed[digits++] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U && digits < sizeof(reversed));

  if ((digits + 1U) > buffer_size)
    return 0U;

  for (u32 i = 0U; i < digits; ++i) {
    buffer[i] = reversed[digits - 1U - i];
  }

  buffer[digits] = '\0';

  return digits;
}

/**
 * Write `value` with a fixed number of decimals (0..6), rounded to nearest.
 * Example: f32_to_string(-3.14159f, buf, 16, 2) -> "-3.14".
 * @return number of characters written (without the terminator), or 0 if
 *         the buffer is too small or the value is not representable
 *         (NaN, infinity, or too large for the chosen precision).
 */
inline u32 f32_to_string(f32 value, char *buffer, u32 buffer_size,
                         u8 decimals = 2U) {
  if (buffer == nullptr || buffer_size == 0U)
    return 0U;

  if (value != value) // NaN
    return 0U;

  if (decimals > 6U)
    decimals = 6U;

  u32 multiplier = 1U;

  for (u8 i = 0U; i < decimals; ++i) {
    multiplier *= 10U;
  }

  const bool negative = value < 0.0f;

  if (negative)
    value = -value;

  // Reject values whose scaled form would not fit in a u32 (also catches inf)
  if (value * static_cast<f32>(multiplier) >= 4.0e9f)
    return 0U;

  // Round before converting to integer
  const u32 scaled =
      static_cast<u32>(value * static_cast<f32>(multiplier) + 0.5f);

  const u32 integer_part = scaled / multiplier;
  const u32 fractional_part = scaled % multiplier;

  u32 index = 0U;

  // Avoid printing "-0.00" for tiny negative values
  if (negative && scaled != 0U) {
    if ((index + 1U) >= buffer_size)
      return 0U;

    buffer[index++] = '-';
  }

  char integer_buffer[11];

  const u32 integer_length =
      u32_to_string(integer_part, integer_buffer, sizeof(integer_buffer));

  if (integer_length == 0U)
    return 0U;

  if ((index + integer_length + 1U) > buffer_size)
    return 0U;

  for (u32 i = 0U; i < integer_length; ++i) {
    buffer[index++] = integer_buffer[i];
  }

  if (decimals == 0U) {
    buffer[index] = '\0';
    return index;
  }

  if ((index + 1U + decimals + 1U) > buffer_size)
    return 0U;

  buffer[index++] = '.';

  // Write the fractional part right to left, keeping leading zeros
  u32 remaining_fraction = fractional_part;

  for (u32 i = 0U; i < decimals; ++i) {
    const u32 position = index + static_cast<u32>(decimals) - 1U - i;

    buffer[position] = static_cast<char>('0' + (remaining_fraction % 10U));

    remaining_fraction /= 10U;
  }

  index += decimals;
  buffer[index] = '\0';

  return index;
}

} // namespace core
