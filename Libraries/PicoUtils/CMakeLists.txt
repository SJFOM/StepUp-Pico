add_library(PicoUtils STATIC
    src/lib/utils.cpp
)

target_include_directories(PicoUtils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(PicoUtils PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_adc
)
