add_subdirectory(crc)
add_subdirectory(cobs)

target_sources(rocket-protocol
    PRIVATE
        codec.c
)

target_link_libraries(rocket-protocol
    PRIVATE
        rp_crc
        rp_cobs
)