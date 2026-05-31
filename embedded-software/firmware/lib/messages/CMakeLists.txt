cmake_minimum_required(VERSION 3.22)
project(messages LANGUAGES C)

# Codegen output sits at firmware/generated/messages/ (sibling tree). Resolved
# from this lib's location so the path holds regardless of which top-level
# CMakeLists invokes us. routing.c is committed alongside its header — if it's
# ever missing (mid-bisect, fresh tree without codegen run), build fails loud.
set(MESSAGES_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../generated/messages")
set(MESSAGES_GENERATED_ROUTING "${MESSAGES_GENERATED_DIR}/routing.c")
set(MESSAGES_GENERATED_PB    "${CMAKE_CURRENT_SOURCE_DIR}/../../generated/proto/messages.pb.c")

if(NOT EXISTS "${MESSAGES_GENERATED_ROUTING}")
    message(FATAL_ERROR
        "messages: generated routing.c missing at ${MESSAGES_GENERATED_ROUTING}. "
        "Run: python -m codegen --registry embedded-software/messages/registry.json")
endif()

# Class B path requires the nanopb runtime + the codegen-generated .pb.c.
# If nanopb_runtime isn't on the build (e.g. unit-test harness), Class B
# stays stubbed — messages.c gates the encode behind MESSAGES_HAVE_NANOPB.
set(MESSAGES_SOURCES
    src/messages.c
    src/crc16.c
    ${MESSAGES_GENERATED_ROUTING}
)
if(TARGET nanopb_runtime AND EXISTS "${MESSAGES_GENERATED_PB}")
    list(APPEND MESSAGES_SOURCES ${MESSAGES_GENERATED_PB})
endif()

add_library(messages STATIC ${MESSAGES_SOURCES})

if(TARGET nanopb_runtime AND EXISTS "${MESSAGES_GENERATED_PB}")
    target_compile_definitions(messages PRIVATE MESSAGES_HAVE_NANOPB=1)
endif()

target_include_directories(messages
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # Expose firmware/ so codegen output can be reached as
        # "generated/messages/<file>" — that's the include convention
        # baked into the codegen-emitted .c/.h files.
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_link_libraries(messages
    PRIVATE
        timestamp
)

if(TARGET nanopb_runtime)
    target_link_libraries(messages PUBLIC nanopb_runtime)
endif()

# Conditionally link CubeMX so any HAL bits pulled in by timestamp resolve
# when this lib is built as part of the firmware tree.
if(TARGET stm32cubemx)
    target_link_libraries(messages PRIVATE stm32cubemx)
endif()

set_target_properties(messages PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
