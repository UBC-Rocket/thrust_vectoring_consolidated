cmake_minimum_required(VERSION 3.22)
project(messages_tests LANGUAGES C)

enable_testing()

# Unity stays as a built library.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../unity ${CMAKE_BINARY_DIR}/unity)

# We deliberately do NOT add_subdirectory(messages). The lib's CMakeLists
# pulls in timestamp + log_records (firmware deps that require HAL include
# paths to build). For the host-side test we build the source files
# directly so we can swap in our fakes for `generated/messages/*` and
# `timestamp/`.
add_executable(test_messages
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/messages.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/crc16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes/generated/messages/routing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes/fake_timestamp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_envelope.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_token_bucket.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_drop_counter.c
)

# Order matters: fakes/ must appear BEFORE the real lib include paths so the
# fake timestamp.h and generated/messages/*.h take precedence.
target_include_directories(test_messages PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(test_messages PRIVATE unity)

set_target_properties(test_messages PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

# Catch lurking warnings that hide bugs.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(test_messages PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
    )
endif()

add_test(NAME messages_tests COMMAND test_messages)
