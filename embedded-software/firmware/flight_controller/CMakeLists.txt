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
if(USE_DYNAMIXEL_SERVO)
    set(CM7_SERVO_CMAKE_ARG -DUSE_DYNAMIXEL_SERVO=ON)
else()
    set(CM7_SERVO_CMAKE_ARG "")
endif()

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Include external project file
include("mx-generated.cmake")

