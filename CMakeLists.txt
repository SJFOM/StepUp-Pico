cmake_minimum_required(VERSION 3.14)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Set project data
set(PROJECT_NAME "STEPUP_PICO")
set(BUILD_NUMBER "0")

# Set app name(s) and version(s)
set(APP_NAME "StepUp")
set(APP_VERSION_NUMBER "0.0.1")

# Set board revision
set(PCB_REVISION 2)

# Specify the app(s) source code
set(APP_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/App-StepUp")

# Sam - create references to additional folders
set(TMC_API_CODE_DIRECTORY "${CMAKE_SOURCE_DIR}/Libraries/TMC_API")
set(INTERFACE_CODE_DIRECTORY "${CMAKE_SOURCE_DIR}/Interface")

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
