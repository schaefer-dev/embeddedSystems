set(CMAKE_SYSTEM_NAME Generic)

find_program(AVR_GCC     avr-gcc)
find_program(AVR_GXX     avr-g++)
find_program(AVR_OBJCOPY avr-objcopy)
find_program(AVR_SIZE    avr-size)

find_program(AVRDUDE     avrdude)

set(CMAKE_C_COMPILER   ${AVR_GCC})
set(CMAKE_CXX_COMPILER ${AVR_GXX})

include_directories("${AVR_LIBC}/include")
link_directories("${AVR_LIBC}/lib")

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

set(CMAKE_C_FLAGS "-Os -Wall -std=gnu11 -Wl,--gc-sections")
set(CMAKE_CXX_FLAGS "-Os -Wall -std=c++14 -Wl,--gc-sections -fno-threadsafe-statics")

set(CMAKE_CXX_STANDARD 14)

function(_avr_configure_target TARGET MCU FREQ)
    set_target_properties(${TARGET} PROPERTIES
            COMPILE_FLAGS "-mmcu=${MCU} -Os  -fno-threadsafe-statics -DF_CPU=${FREQ}"
            LINK_FLAGS "-mmcu=${MCU}")
endfunction(_avr_configure_target)


function(add_avr_executable EXECUTABLE MCU FREQ)
    set(HEX "${EXECUTABLE}.hex")
    set(EEP "${EXECUTABLE}.eep")

    add_executable(${EXECUTABLE} ${ARGN})
    _avr_configure_target(${EXECUTABLE} ${MCU} ${FREQ})

    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
            COMMAND ${AVR_SIZE} --mcu=${MCU} --format=avr ${EXECUTABLE})

    add_custom_command(OUTPUT ${HEX}_output
            COMMAND ${AVR_OBJCOPY} -O ihex -R.eeprom ${EXECUTABLE} ${HEX}
            DEPENDS ${EXECUTABLE})

    add_custom_command(OUTPUT ${EEP}_output
            COMMAND ${AVR_OBJCOPY} -O ihex -j.eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 ${EXECUTABLE} ${EEP}
            DEPENDS ${EXECUTABLE})

    add_custom_target(${HEX} ALL DEPENDS ${HEX}_output)
    add_custom_target(${EEP} ALL DEPENDS ${EEP}_output)

    add_custom_target(
            ${EXECUTABLE}_size_info
            ${AVR_SIZE} --mcu=${MCU} --format=avr ${EXECUTABLE}
            DEPENDS ${EXECUTABLE}
    )

    add_custom_target(
            ${EXECUTABLE}_upload_flash
            ${AVRDUDE} ${AVRDUDE_OPTIONS} -p ${MCU} -U flash:w:${HEX}
            DEPENDS ${HEX}
    )

    add_custom_target(
            ${EXECUTABLE}_upload_eeprom
            ${AVRDUDE} ${AVRDUDE_OPTIONS} -p ${MCU} -U eeprom:w:${EEP}
            DEPENDS ${EEP}
    )
endfunction(add_avr_executable)


function(add_avr_library LIBRARY MCU FREQ)
    add_library(${LIBRARY} STATIC ${ARGN})
    _avr_configure_target(${LIBRARY} ${MCU} ${FREQ})
endfunction(add_avr_library)
