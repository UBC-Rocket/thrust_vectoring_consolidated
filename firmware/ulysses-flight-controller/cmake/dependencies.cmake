include(FetchContent)

FetchContent_Declare(
    printf
    GIT_REPOSITORY https://github.com/eyalroz/printf.git
    GIT_TAG f1b728cbd5c6e10dc1f140f1574edfd1ccdcbedb # 6.3.0
    GIT_SHALLOW ON
)

FetchContent_MakeAvailable(printf)

# Library requires a native 64-bit integer type, so must be be compiled
# with at least C99 standard. But it defaults to C90 for GNU and Clang.
# So we manually bump up the C standard version.
set_target_properties(printf PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
