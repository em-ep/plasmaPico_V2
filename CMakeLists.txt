# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(plasmaPico_V2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(plasmaPico_V2 plasmaPico_V2.c )

pico_set_program_name(plasmaPico_V2 "plasmaPico_V2")
pico_set_program_version(plasmaPico_V2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(plasmaPico_V2 0)
pico_enable_stdio_usb(plasmaPico_V2 1)

# Add the standard library to the build
target_link_libraries(plasmaPico_V2
    pico_stdlib
    hardware_pio
    hardware_pwm
    hardware_adc
    hardware_sync)

# Create C header file with the name <pio program>.pio.h
pico_generate_pio_header(${PROJECT_NAME}
        ${CMAKE_CURRENT_LIST_DIR}/pinsToggle.pio
)

# Add the standard include files to the build
target_include_directories(plasmaPico_V2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Debugging
# Set default debug level (0-4)
#set(DEBUG_LEVEL 4 CACHE STRING "Debug level (0=OFF, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG)")

# Add to compiler definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    DEBUG_LEVEL=${DEBUG_LEVEL}
)

pico_add_extra_outputs(plasmaPico_V2)

