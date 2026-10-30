add_library(hal
  ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32f4/drv_gpio_stm32f4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32f4/drv_time_stm32f4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32f4/drv_i2c_stm32f4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32f4/drv_uart_stm32f4.cpp
)

target_include_directories(hal PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(hal PUBLIC
  stm32f4xx
)
