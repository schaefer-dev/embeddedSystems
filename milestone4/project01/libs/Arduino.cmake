set(ARDUINO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/arduino)

file(GLOB_RECURSE ARDUINO_C_FILES ${ARDUINO_DIR}/*.c)
file(GLOB_RECURSE ARDUINO_CXX_FILES ${ARDUINO_DIR}/*.cpp)


function(add_arduino_core CORE VARIANT MCU FREQ)
    add_avr_library(${CORE} ${MCU} ${FREQ} ${ARDUINO_CXX_FILES} ${ARDUINO_C_FILES})

    target_include_directories(${CORE} PUBLIC ${ARDUINO_DIR}/variants/${VARIANT})
    target_include_directories(${CORE} PUBLIC ${ARDUINO_DIR})
endfunction(add_arduino_core)


function(arduino_configure_target TARGET VARIANT CORE)
    target_include_directories(${TARGET} PUBLIC
            ${ARDUINO_DIR}
            ${ARDUINO_DIR}/variants/${VARIANT})

    target_link_libraries(${TARGET} ${CORE})
    target_compile_definitions(${TARGET} PUBLIC ARDUINO=1)
endfunction(arduino_configure_target)


function(add_arduino_library LIBRARY VARIANT CORE MCU FREQ)
    add_avr_library(${LIBRARY} ${MCU} ${FREQ} ${ARGN})
    arduino_configure_target(${LIBRARY} ${VARIANT} ${CORE})
endfunction(add_arduino_library)

function(add_arduino_executable EXECUTABLE VARIANT CORE MCU FREQ)
    add_avr_executable(${EXECUTABLE} ${MCU} ${FREQ} ${ARGN})
    arduino_configure_target(${EXECUTABLE} ${VARIANT} ${CORE})
endfunction(add_arduino_executable)
