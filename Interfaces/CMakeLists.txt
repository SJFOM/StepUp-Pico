add_library(ControlInterface STATIC
    ControlInterface.cpp
)

target_include_directories(ControlInterface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ControlInterface PUBLIC
    PicoUtils
)
