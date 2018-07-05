set(ZUMO_MCU  atmega32u4)
set(ZUMO_FREQ 16000000L)

set(LIBZUMO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libzumo)

file(GLOB_RECURSE LIBZUMO_FILES ${LIBZUMO_DIR}/*.cpp)

include_directories(${LIBZUMO_DIR})

add_arduino_core(_zumo_core leonardo ${ZUMO_MCU} ${ZUMO_FREQ})
target_compile_definitions(_zumo_core PUBLIC
        USB_VID=0x1ffb
        USB_PID=0x2300
        USB_MANUFACTURER="Pololu A-Star 32U4"
        USB_PRODUCT="Pololu Corporation")

add_arduino_library(_zumo leonardo _zumo_core ${ZUMO_MCU} ${ZUMO_FREQ}
        ${LIBZUMO_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/Wire/src/Wire.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/Wire/src/utility/twi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/SPI/src/SPI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/DigitalIO/src/PinIO.cpp)
target_include_directories(_zumo PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/Wire/src
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/SPI/src
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/DigitalIO/src)


function(add_zumo_executable EXECUTABLE)
    add_arduino_executable(${EXECUTABLE} leonardo _zumo_core ${ZUMO_MCU} ${ZUMO_FREQ} ${ARGN})

    target_link_libraries(${EXECUTABLE} _zumo)
    target_include_directories(${EXECUTABLE} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/libs/rf24
            ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/Wire/src
            ${CMAKE_CURRENT_SOURCE_DIR}/libs/libraries/SPI/src)
endfunction()
