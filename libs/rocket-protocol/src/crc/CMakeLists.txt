add_library(rp_crc)

set_target_properties(rp_crc PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

target_sources(rp_crc
    PRIVATE
        crc16.c
)

target_link_libraries(rp_crc
    PUBLIC
        rp_library_interface
)