set(APP_NAME attitude_indicator)

add_executable(${APP_NAME}
  main.cpp
  ${PROJECT_SOURCE_DIR}/stm32f4xx/startup/startup_stm32f407vetx.s
  ${PROJECT_SOURCE_DIR}/stm32f4xx/startup/system_stm32f4.cpp
)

target_include_directories(${APP_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${APP_NAME} PRIVATE
  hal
)

set_target_properties(${APP_NAME} PROPERTIES
  SUFFIX ".elf"
)

target_link_options(${APP_NAME} PRIVATE
  -T${PROJECT_SOURCE_DIR}/stm32f4xx/ld/STM32F407VETX_FLASH.ld
  -Wl,-Map=$<TARGET_FILE_DIR:${APP_NAME}>/${APP_NAME}.map
)

# Raw binary for st-flash / drag-and-drop programmers, plus a size report
add_custom_command(TARGET ${APP_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${APP_NAME}>
          $<TARGET_FILE_DIR:${APP_NAME}>/${APP_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${APP_NAME}>
  VERBATIM
)
