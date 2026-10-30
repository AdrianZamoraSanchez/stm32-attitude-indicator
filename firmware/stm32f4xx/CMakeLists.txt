add_library(stm32f4xx INTERFACE)

# -----------------------------
# CPU flags
# -----------------------------
set(STM32_CPU_FLAGS
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
)

# -----------------------------
# Compile flags
# -----------------------------
target_compile_options(stm32f4xx INTERFACE
  ${STM32_CPU_FLAGS}

  -Wall
  -Wextra
  -Wpedantic

  -ffunction-sections
  -fdata-sections

  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>
)

# -----------------------------
# Link flags
# -----------------------------
target_link_options(stm32f4xx INTERFACE
  ${STM32_CPU_FLAGS}

  -Wl,--gc-sections
  -Wl,--print-memory-usage
)

target_include_directories(stm32f4xx INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_precompile_headers(stm32f4xx INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/platform/platform.h
)

target_compile_definitions(stm32f4xx INTERFACE
  STM32F407xx
)