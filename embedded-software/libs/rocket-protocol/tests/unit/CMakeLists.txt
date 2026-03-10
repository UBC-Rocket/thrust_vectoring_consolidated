include(nanopb-generate)

find_package(unity REQUIRED)

set(UNIT_TEST_CODEGEN_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated/tests/unit")
set(NANOPB_GENERATE_DEBUG ON)

find_program(PROTOC_EXE
    NAMES protoc
    DOC "The Google Protocol Buffers Compiler"
    PATHS
        ${nanopb_SOURCE_DIR}/generator-bin
        ${nanopb_SOURCE_DIR}/generator
    NO_DEFAULT_PATH
)

function(add_unity_test)
    cmake_parse_arguments(arg
        ""
        "NAME"
        "SOURCES;LIBRARIES;INCLUDE_DIRECTORIES"
        ${ARGN}
    )

    set(test_target "test_${arg_NAME}")

    add_executable(${test_target})

    target_sources(${test_target}
        PRIVATE
            ${arg_SOURCES}
    )

    target_include_directories(${test_target}
        PRIVATE
            ${arg_INCLUDE_DIRECTORIES}
    )

    target_link_libraries(${test_target}
        PRIVATE
            unity::framework
            ${arg_LIBRARIES}
    )

    add_test(
        NAME ${arg_NAME}
        COMMAND ${test_target}
    )
endfunction()

nanopb_generate(
    PROTOC_EXE ${PROTOC_EXE}
    IMPORT_DIRS ${CMAKE_CURRENT_LIST_DIR}
    PROTOS
        proto/codec_test_data.proto
    PROTOC_OUT_DIR ${UNIT_TEST_CODEGEN_DIRECTORY}
    OUT_VAR PROTO_GENERATED_SOURCES
)

add_unity_test(
    NAME "codec"
    SOURCES
        codec/test_codec.c
        ${PROTO_GENERATED_SOURCES}
    LIBRARIES
        rocket-protocol::protocol
    INCLUDE_DIRECTORIES
        ${UNIT_TEST_CODEGEN_DIRECTORY}
)

add_unity_test(
    NAME "crc16"
    SOURCES
        crc/test_crc16.c
    LIBRARIES
        rp_crc
)

add_unity_test(
    NAME "cobs_encode"
    SOURCES
        cobs/test_cobs_encode.c
    LIBRARIES
        rp_cobs
)

add_unity_test(
    NAME "cobs_decode"
    SOURCES
        cobs/test_cobs_decode.c
    LIBRARIES
        rp_cobs
)

add_unity_test(
    NAME "cobs_encode_decode"
    SOURCES
        cobs/test_cobs_encode_decode.c
    LIBRARIES
        rp_cobs
)

add_unity_test(
    NAME "cobs_get_max_encoded_size"
    SOURCES
        cobs/test_cobs_get_max_encoded_size.c
    LIBRARIES
        rp_cobs
)