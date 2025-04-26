# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.14)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Set project data
set(PROJECT_NAME "STEPUP_PICO")

# Set app name(s) and version(s)
set(APP_NAME "StepUp")
set(APP_VERSION_NUMBER "1.0.0")

# Set board revision
set(PCB_REVISION 3)

# Specify the app(s) source code
set(APP_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/App-StepUp")

# Create references to additional folders
set(TMC_API_CODE_DIRECTORY "${CMAKE_SOURCE_DIR}/Libraries/TMC_API")
set(INTERFACE_CODE_DIRECTORY "${CMAKE_SOURCE_DIR}/Interfaces")
set(LIBRARIES_CODE_DIRECTORY "${CMAKE_SOURCE_DIR}/Libraries")

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
    set(LAST_BUILD_NUMBER 1)
endif()

# Check if the commit hash has changed
if(NOT "${GIT_COMMIT_HASH}" STREQUAL "${LAST_COMMIT_HASH}")
    set(BUILD_NUMBER 0)
else()
    math(EXPR BUILD_NUMBER "${LAST_BUILD_NUMBER} + 1")
    set(BUILD_NUMBER_OUTPUT "${BUILD_NUMBER}")
endif()

# Write the new build number to the file
file(WRITE ${BUILD_NUMBER_FILE} "${GIT_COMMIT_HASH}:${BUILD_NUMBER}")

set(BUILD_NUMBER_OUTPUT "${GIT_COMMIT_HASH}:${BUILD_NUMBER}")

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_NAME}")
add_compile_definitions(APP_VERSION="${APP_VERSION_NUMBER}")
add_compile_definitions(PCB_REVISION=${PCB_REVISION})
add_compile_definitions(BUILD_NUM="${BUILD_NUMBER_OUTPUT}")

# Set app-side debugging "ON" or "OFF"
set(DO_DEBUG "ON")

# Set env variable 'PICO_SDK_PATH' to the local Pico SDK
# Comment out the set() if you have a global copy of the
# SDK set and $PICO_SDK_PATH defined in your $PATH
set(ENV{PICO_SDK_PATH} "${CMAKE_SOURCE_DIR}/pico-sdk")

# Set flags and directory variables
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DTSDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DTSDEBUG")
set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/Config")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")


# Name the project
project(${PROJECT_NAME}
    LANGUAGES C CXX ASM
    DESCRIPTION "FreeRTOS-based RP2040 Stepper motor driver application"
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the Pico SDK
pico_sdk_init()

# Add FreeRTOS as a library
add_library(FreeRTOS STATIC
    ${FREERTOS_SRC_DIRECTORY}/event_groups.c
    ${FREERTOS_SRC_DIRECTORY}/list.c
    ${FREERTOS_SRC_DIRECTORY}/queue.c
    ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
    ${FREERTOS_SRC_DIRECTORY}/tasks.c
    ${FREERTOS_SRC_DIRECTORY}/timers.c
    ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
)

# Build FreeRTOS
target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_CFG_DIRECTORY}/
    ${FREERTOS_SRC_DIRECTORY}/include
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
)

# Include the apps' source code
add_subdirectory(${APP_SRC_DIRECTORY}) # App-StepUp
add_subdirectory(${LIBRARIES_CODE_DIRECTORY}) # Libraries
add_subdirectory(${INTERFACE_CODE_DIRECTORY}) # Interfaces