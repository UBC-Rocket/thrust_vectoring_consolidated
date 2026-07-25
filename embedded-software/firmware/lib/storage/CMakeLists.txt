cmake_minimum_required(VERSION 3.22)
project(storage LANGUAGES C)

set(STORAGE_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/../../generated/storage")

if(NOT EXISTS "${STORAGE_GENERATED}/storage_data.c")
    message(FATAL_ERROR
        "storage: generated/storage/storage_data.c missing. "
        "Run: python -m codegen --registry embedded-software/storage/registry.json")
endif()

add_library(storage STATIC
    src/crc32.c
    src/storage.c
    ${STORAGE_GENERATED}/storage_data.c
)

target_include_directories(storage
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # Expose firmware/ so generated/storage/*.h is reachable as
        # "generated/storage/<file>" — same pattern lib/messages uses.
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(TARGET stm32cubemx)
    target_link_libraries(storage PRIVATE stm32cubemx)
endif()

set_target_properties(storage PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
