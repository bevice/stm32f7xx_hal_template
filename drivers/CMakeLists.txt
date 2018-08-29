cmake_minimum_required(VERSION 3.8 FATAL_ERROR)


include(hal_defines.cmake)
include(ll_stm32f7xx.cmake)
include(hal_stm32f7xx.cmake)



SET(CMSIS_INCLUDE_DIRECTORIES
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Include/
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Device/ST/STM32F7xx/Include/
        )

set(STM32F7XX_LL_INCLUDE_DIRECTORIES
        ${CMAKE_CURRENT_SOURCE_DIR}/STM32F7xx_HAL_Driver/Inc
        )
set(STM32F7XX_HAL_INCLUDE_DIRECTORIES
        ${CMAKE_CURRENT_SOURCE_DIR}/STM32F7xx_HAL_Driver/Inc
        )





add_library(stm32f7xx_ll
        ${STM32F7XX_LL_HEADERS}
        ${STM32F7XX_LL_SOURCES}
        )

add_library(stm32f7xx_hal
        ${STM32F7XX_HAL_HEADERS}
        ${STM32F7XX_HAL_SOURCES}
        )
target_link_libraries(stm32f7xx_hal
        stm32f7xx_ll
        )


target_include_directories(stm32f7xx_hal
        PUBLIC STM32F7xx_HAL_Driver/Inc
        PUBLIC ${CMSIS_INCLUDE_DIRECTORIES}
        )
target_include_directories(stm32f7xx_ll
        PUBLIC STM32F7xx_HAL_Driver/Inc
        PUBLIC ${CMSIS_INCLUDE_DIRECTORIES}
        )