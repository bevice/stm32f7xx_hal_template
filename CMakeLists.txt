# Нужен cmake >=3.5
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)


add_definitions(-DSTM32F746xx)

include(stm32f7.cmake)

PROJECT(stm32f7_hal_template)

## float version of printf()
#SET(PRINTF_FLOAT TRUE)


include_directories(${CMSIS_INCLUDE_DIRECTORIES})


include(drivers/hal_defines.cmake)
add_subdirectory(drivers)

####################################################

SET(STARTUP_SOURCES
        startup/system_stm32f7xx.c
        startup/startup_stm32f746xx.s
        )

SET(PROJECT_SOURCES
        src/main.c
        src/main.h
        src/clockerf7/clock.c
        src/clockerf7/clock.h
        )



#скрипт линковки и где его искать
SET(LD_SCRIPT STM32F746NGHx_FLASH.ld)
SET(LD_SCRIPTS_DIR ${PROJECT_SOURCE_DIR}/ld-scripts/)
SET(CMAKE_EXE_LINKER_FLAGS "-L${LD_SCRIPTS_DIR} -T${LD_SCRIPT} ${CMAKE_EXE_LINKER_FLAGS}")


# Добавляем пути поиска заголовочных файлов
INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/src/
)

# arm_math
add_definitions(-DARM_MATH_CM3)
link_directories(drivers/CMSIS/Lib/GCC)


# Собираем исходники пректа, модули, и т.д. в elf
ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf
        ${STARTUP_SOURCES}
        ${PROJECT_SOURCES}
        )

target_link_libraries(${CMAKE_PROJECT_NAME}.elf stm32f7xx_ll)
target_link_libraries(${CMAKE_PROJECT_NAME}.elf stm32f7xx_hal)
#target_link_libraries(${CMAKE_PROJECT_NAME}.elf arm_cortexM3l_math)
target_link_libraries(${CMAKE_PROJECT_NAME}.elf m)



# Конвертируем elf в hex и bin

ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Oihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex)
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Obinary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin)
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_SIZE} ARGS --format=sysv ${CMAKE_PROJECT_NAME}.elf)
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_SIZE} ARGS --format=berkeley ${CMAKE_PROJECT_NAME}.elf)

if(PRINTF_FLOAT)
    message(STATUS "Using float version of printf()")
    SET(CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
endif()

