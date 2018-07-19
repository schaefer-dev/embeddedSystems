cmake_minimum_required(VERSION 3.3)

# include configuration
include(Config.cmake)

# Optimize compilaton for size
# SET(SIZE "-fdata-sections -ffunction-sections -Wl, --gc-sections")
SET(SIZE "-fdata-sections -ffunction-sections")
add_definitions(${SIZE})

set(CMAKE_TOOLCHAIN_FILE libs/Toolchain.cmake)


project(EmbeddedSystems18)

include(libs/Arduino.cmake)
include(libs/Pi.cmake)
include(libs/Zumo.cmake)


# Scout Executable
add_pi_executable(scout
        src/scout/main.cpp src/scout/ScoutState.cpp src/utils/Coordinates.cpp src/scout/ScoutSerial.cpp src/scout/ScoutMonitor.cpp src/utils/Utility.cpp src/scout/ScoutRF.cpp src/scout/ScoutLineSensors.cpp src/scout/spi_s.cpp src/scout/platform_s.cpp src/scout/adc.cpp)

target_compile_definitions(scout PUBLIC SCOUT=1)


# Collector Executable
add_zumo_executable(collector
        src/collector/main.cpp src/utils/Coordinates.cpp src/collector/CollectorState.cpp src/collector/CollectorMonitor.cpp src/collector/CollectorSPI.cpp src/utils/Utility.cpp src/collector/CollectorRF.cpp src/collector/CollectorLineSensors.cpp src/collector/spi_c.cpp src/collector/platform_c.cpp)
