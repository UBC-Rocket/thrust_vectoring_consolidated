cmake_minimum_required(VERSION 3.22)
project(gnss-protocol LANGUAGES C)

add_library(gnss-protocol INTERFACE)
target_include_directories(gnss-protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
