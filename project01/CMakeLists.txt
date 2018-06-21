cmake_minimum_required(VERSION 3.3)

# include configuration
include(Config.cmake)

set(CMAKE_TOOLCHAIN_FILE libs/Toolchain.cmake)


project(EmbeddedSystems18)

include(libs/Arduino.cmake)
include(libs/Pi.cmake)
include(libs/Zumo.cmake)


# Scout Executable
add_pi_executable(scout
        src/scout/main.cpp src/scout/ScoutState.cpp src/collector/Coordinates.cpp src/scout/ScoutSPI.cpp src/scout/ScoutSerial.cpp src/scout/ScoutMonitor.cpp src/scout/Utility.cpp)


# Collector Executable
add_zumo_executable(collector
        src/collector/main.cpp src/collector/Coordinates.cpp src/collector/CollectorState.cpp src/collector/CollectorMonitor.cpp src/collector/CollectorSPI.cpp src/scout/Utility.cpp)
