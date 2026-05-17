cmake_minimum_required(VERSION 3.22)
project(collections LANGUAGES C)

add_library(collections INTERFACE)
target_include_directories(collections INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
