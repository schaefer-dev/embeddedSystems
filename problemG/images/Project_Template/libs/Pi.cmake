set(PI_MCU  atmega328p)
set(PI_FREQ 20000000L)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/libpololu)

function(add_pi_executable EXECUTABLE)
    add_avr_executable(${EXECUTABLE} ${PI_MCU} ${PI_FREQ} ${ARGN})

    target_link_libraries(${EXECUTABLE} pololu)
    target_include_directories(${EXECUTABLE} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/libs/libpololu/pololu)
endfunction()
