cmake_minimum_required(VERSION 3.15.0)

project(mcu1)

enable_language(C CXX)

set(CMAKE_C_STANDARD 11)

set(PROJECT_FILES
        drivers/Inc/stm32f407xx.h
        drivers/Src/stm32f407xx.cpp
        drivers/Inc/stm32f407xx_gpio_driver.h
        drivers/Inc/stm32f407xx_spi_driver.h
        drivers/Inc/stm32f407xx_usart_driver.h
        drivers/Inc/stm32f407xx_rcc_driver.h
        drivers/Inc/stm32f407xx_i2c_driver.h
        drivers/Src/stm32f407xx_gpio_driver.cpp
        drivers/Src/stm32f407xx_spi_driver.cpp
        drivers/Src/stm32f407xx_usart_driver.cpp
        drivers/Src/stm32f407xx_rcc_driver.cpp
        drivers/Src/stm32f407xx_i2c_driver.cpp
        modules/epaper.h
        modules/epaper.cpp
        modules/GUI_Paint.h
        modules/GUI_Paint.cpp
        modules/Fonts/fonts.h
        modules/Fonts/font8.cpp
        modules/Fonts/font12.cpp
        modules/Fonts/font16.cpp
        modules/Fonts/font20.cpp
        modules/Fonts/font24.cpp
        modules/ImageData.h
        modules/ImageData.cpp
        modules/barcode.h
        modules/barcode.cpp
#        modules/epaper/Inc/EPD_2in66.h
#        modules/epaper/Src/EPD_2in66.cpp
#        modules/epaper/Fonts/font8.cpp
#        modules/epaper/Fonts/font12.cpp
#        modules/epaper/Fonts/font16.cpp
#        modules/epaper/Fonts/font20.cpp
#        modules/epaper/Fonts/font24.cpp
#        modules/epaper/Fonts/fonts.h
#        modules/epaper/GUI/GUI_Paint.cpp
#        modules/epaper/GUI/GUI_Paint.h
        Src/exp19_i2c_master_rx_it.cpp
        stm32_startup.c
        )

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} ${PROJECT_FILES})

target_include_directories(${EXECUTABLE} PRIVATE
        drivers/Inc
        drivers/Src
        modules
        modules/Fonts
        Src
        )

set(MACH cortex-m4)

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=${MACH}
        -mthumb
        -mfloat-abi=soft
        -Wall
        -fno-exceptions
        $<$<CONFIG:Debug>:-O0>
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