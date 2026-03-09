cmake_minimum_required(VERSION 3.22)

project(unity LANGUAGES C)

add_library(unity STATIC src/unity.c)
target_include_directories(unity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_target_properties(unity PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
