cmake_minimum_required(VERSION 3.22)
include(ExternalProject)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Set the project name
set(CMAKE_PROJECT_NAME ulysses-fc)

option(USE_DYNAMIXEL_SERVO "Use Dynamixel XM430 instead of Feetech on UART8" OFF)
option(BARO_RELAX_CRC "Relax MS5611 PROM CRC gate (bring-up boards)" OFF)

# -D args forwarded to the CM4 ExternalProject sub-build (consumed as
# ${CM4_SERVO_CMAKE_ARG} in mx-generated.cmake). Named for its original
# servo-only use; now also carries other CM4 bring-up toggles. Top-level -D
# options don't reach the per-core sub-builds automatically — each must be
# forwarded here explicitly.
set(CM4_SERVO_CMAKE_ARG "")
if(USE_DYNAMIXEL_SERVO)
    list(APPEND CM4_SERVO_CMAKE_ARG -DUSE_DYNAMIXEL_SERVO=ON)
endif()
if(BARO_RELAX_CRC)
    list(APPEND CM4_SERVO_CMAKE_ARG -DBARO_RELAX_CRC=ON)
endif()

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Include external project file
include("mx-generated.cmake")

