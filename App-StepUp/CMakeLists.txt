cmake_minimum_required(VERSION 3.14)

# Should we set app-level debugging?
# NOTE Equivalent of `#define DEBUG 1`
if(${DO_DEBUG})
    add_compile_definitions(DEBUG=1)
    message(STATUS "App-side debugging enabled for ${APP_NAME}")
else()
    message(STATUS "App-side debugging disabled for ${APP_NAME}")
endif()

message(STATUS "Compiling for PCB Revision ${PCB_REVISION}")

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_NAME}")
add_compile_definitions(APP_VERSION="${APP_VERSION_NUMBER}")
add_compile_definitions(PCB_REVISION=${PCB_REVISION})

# Include app source code file(s)
file(GLOB APP_SOURCES
    "${APP_SRC_DIRECTORY}/*.c*"
    "${APP_SRC_DIRECTORY}/src/*.c*"
    "${TMC_API_CODE_DIRECTORY}/ic/*.c*"
    "${TMC_API_CODE_DIRECTORY}/helpers/*.c*"
)

add_executable(${APP_NAME}
    ${APP_SOURCES}
)

# Link to built libraries
target_link_libraries(${APP_NAME} LINK_PUBLIC
    pico_stdlib
    hardware_adc
    hardware_pwm
    FreeRTOS)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_NAME} 0)
pico_enable_stdio_uart(${APP_NAME} 1)

# Enable extra build products
pico_add_extra_outputs(${APP_NAME})

# This section of code creates a build version of the format "GIT_COMMIT_HASH:BUILD_NUMBER"
# where GIT_COMMIT_HASH is the short hash of the current git commit and BUILD_NUMBER is
# incremented for every build. This is useful for tracking the version of the firmware
# that is currently running on the device. The BUILD_NUMBER is stored in a file named
# "build_number.txt" in the build directory. The file contains the last commit hash and
# the last build number, separated by a colon. If the commit hash has changed since the
# last build, the BUILD_NUMBER is reset to 0. Otherwise, it is incremented by 1.

# Increment the build number for every build
set(BUILD_NUMBER_FILE "${CMAKE_BINARY_DIR}/build_number.txt")

# Function to get the current git commit hash
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Read the existing build number file if it exists
if(EXISTS ${BUILD_NUMBER_FILE})
    file(READ ${BUILD_NUMBER_FILE} BUILD_NUMBER_CONTENT)
    string(REGEX MATCH "([^:]+):([0-9]+)" _ ${BUILD_NUMBER_CONTENT})
    set(LAST_COMMIT_HASH ${CMAKE_MATCH_1})
    set(LAST_BUILD_NUMBER ${CMAKE_MATCH_2})
else()
    set(LAST_COMMIT_HASH "")
    set(LAST_BUILD_NUMBER 0)
endif()

# Check if the commit hash has changed
if(NOT "${GIT_COMMIT_HASH}" STREQUAL "${LAST_COMMIT_HASH}")
    set(BUILD_NUMBER 0)
else()
    math(EXPR BUILD_NUMBER "${LAST_BUILD_NUMBER} + 1")
endif()

# Write the new build number to the file
file(WRITE ${BUILD_NUMBER_FILE} "${GIT_COMMIT_HASH}:${BUILD_NUMBER}")

# Make the build number accessible to the compiler
add_compile_definitions(BUILD_NUM="${GIT_COMMIT_HASH}:${BUILD_NUMBER}")

message(STATUS "Build revision ${GIT_COMMIT_HASH}:${BUILD_NUMBER}")
