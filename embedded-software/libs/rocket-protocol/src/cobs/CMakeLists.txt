add_library(rp_cobs)

set_target_properties(rp_cobs PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

target_sources(rp_cobs
    PRIVATE
        cobs.c
)

target_link_libraries(rp_cobs
    PUBLIC
        rp_library_interface
)