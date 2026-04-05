cmake_minimum_required(VERSION 3.22)

project(ulysses_fc_tests LANGUAGES C)

enable_testing()

set(FW_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libs)

add_subdirectory(${FW_LIBS_DIR}/sensors ${CMAKE_BINARY_DIR}/sensors)
add_subdirectory(${FW_LIBS_DIR}/unity ${CMAKE_BINARY_DIR}/unity)

add_executable(test_ulysses_fc
    test_main.c
    test_support/fake_stm32_runtime.c
    test_icm40609_device.c
    ../Core/Src/spi_drivers/icm40609_device.c
    ../Core/Src/spi_drivers/SPI_device_common.c
)

target_include_directories(test_ulysses_fc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes
    ${CMAKE_CURRENT_SOURCE_DIR}/test_support
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc/spi_drivers
)

target_link_libraries(test_ulysses_fc PRIVATE sensors unity m)

add_test(NAME ulysses_fc_tests COMMAND test_ulysses_fc)
