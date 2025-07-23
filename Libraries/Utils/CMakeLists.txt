add_library(Utils STATIC
    src/lib/utils.cpp
)

target_include_directories(Utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)