cmake_minimum_required(VERSION 3.15.0)

project(mcu1)

enable_language(C CXX)

set(CMAKE_C_STANDARD 99)

set(PROJECT_FILES
        drivers/Inc/stm32f407xx.h
        drivers/Inc/stm32f407xx_gpio_driver.h
        drivers/Src/stm32f407xx_gpio_driver.cpp
        stm32_startup.c

        Src/main.cpp
        )

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} ${PROJECT_FILES})

target_include_directories(${EXECUTABLE} PRIVATE
        drivers/Inc
        drivers/Src
        Src
        )

set(MACH cortex-m4)

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=${MACH}
        -mthumb
        -mfloat-abi=soft
        -Wall
        -fno-exceptions
        $<$<CONFIG:Debug>:-o0>
        )

target_link_options(${EXECUTABLE} PRIVATE
        -T ${CMAKE_SOURCE_DIR}/stm32_ls.ld
#        -nostdlib
        -mcpu=${MACH}
        -mthumb
        -mfloat-abi=soft
        --specs=nosys.specs
        -Wl,-Map=${PROJECT_NAME}.map
        )