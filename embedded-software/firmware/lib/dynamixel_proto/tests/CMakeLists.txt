cmake_minimum_required(VERSION 3.22)

project(dynamixel_proto_tests LANGUAGES C)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/dynamixel_proto)

add_executable(test_dxl_packet
    test_dxl_packet.c
)

target_link_libraries(test_dxl_packet PRIVATE dynamixel_proto m)

add_test(NAME dynamixel_proto_tests COMMAND test_dxl_packet)
