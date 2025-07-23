add_library(AxisToActuatorEngine STATIC
    src/lib/axis_to_actuator_engine.cpp
)

target_include_directories(AxisToActuatorEngine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)