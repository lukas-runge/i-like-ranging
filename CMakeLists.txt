cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350)
# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

include(VL53L1X-C-API-for-Raspberry-Pi-Pico/library/import.cmake)

project(i_like_ranging)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

if (TARGET tinyusb_device)

    # rest of your project
    add_executable(${PROJECT_NAME}
        src/main.cpp
    )

    # Add pico_stdlib library which aggregates commonly used features
    target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_multicore
        hardware_i2c
        VL53L1X_pico_api
    )

    # enable usb output, disable uart output
    pico_enable_stdio_usb(${PROJECT_NAME} 1)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)

    # create map/bin/hex/uf2 file in addition to ELF.
    pico_add_extra_outputs(${PROJECT_NAME})

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()