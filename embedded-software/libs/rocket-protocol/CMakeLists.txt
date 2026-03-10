cmake_minimum_required(VERSION 3.22)

project(
    rocket-protocol
    DESCRIPTION "UBC Rocket shared protocol library"
    LANGUAGES C
)

option(ROCKET_PROTOCOL_BUILD_TESTS "Build tests for the rocket protocol library" OFF)

# Generate compile_commands.json for development tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add our own CMake files to the module search path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Linkable interface library to share common include paths with sub-libraries
add_library(rp_library_interface INTERFACE)
target_include_directories(rp_library_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Vendored nanopb runtime (pb_encode, pb_decode, pb_common)
add_library(nanopb-runtime STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/nanopb/pb_encode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/nanopb/pb_decode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/nanopb/pb_common.c
)
target_include_directories(nanopb-runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/nanopb)
set_target_properties(nanopb-runtime PROPERTIES C_STANDARD 11)

# Pre-generated protobuf sources
set(PROTOBUF_GENERATED_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/tvr/common.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/tvr/telemetry.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/tvr/command.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/tvr/status.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/tvr/downlink.pb.c
)

# Define our library target
add_library(rocket-protocol)
add_library("rocket-protocol::protocol" ALIAS rocket-protocol)

set_target_properties(rocket-protocol PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

target_sources(rocket-protocol PRIVATE ${PROTOBUF_GENERATED_SRC})

target_include_directories(rocket-protocol
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/tvr
)

target_link_libraries(rocket-protocol
    PUBLIC
        nanopb-runtime
)

add_subdirectory(src)

if(ROCKET_PROTOCOL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
