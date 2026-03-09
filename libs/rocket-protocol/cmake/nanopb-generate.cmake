set(NANOPB_PROTOC_GENERATE_EXTENSIONS ".pb.c" ".pb.h")
set(NANOPB_OPTIONS_FILE_EXTENSION ".options")

set(NANOPB_PLUGIN_DIR "generator")
set(NANOPB_PLUGIN_NAME "protoc-gen-nanopb")
set(NANOPB_PLUGIN_EXE_WIN32 "protoc-gen-nanopb.bat")
set(NANOPB_PLUGIN_EXE_GENERIC "protoc-gen-nanopb")
set(NANOPB_PLUGIN_DEFAULT_OPTS "--c-style")

if(NOT DEFINED NANOPB_GENERATE_DEBUG)
    set(NANOPB_GENERATE_DEBUG FALSE)
endif()

function(nanopb_generate)
    set(option_arguments)
    set(single_arguments PROTOC_EXE OUT_VAR PROTOC_OUT_DIR)
    set(multi_arguments PROTOS IMPORT_DIRS DEPENDENCIES)

    cmake_parse_arguments(nanopb_generate
        "${option_arguments}"
        "${single_arguments}"
        "${multi_arguments}"
        ${ARGN}
    )

    # FIXME: don't rely on nanopb being brought in by FetchContent
    if(NOT DEFINED nanopb_SOURCE_DIR)
        message(SEND_ERROR "${CMAKE_CURRENT_FUNCTION}(...) called without nanopb library")
        return()
    endif()

    #
    # Validate arguments
    #

    if(NOT DEFINED nanopb_generate_PROTOC_EXE)
        message(SEND_ERROR "${CMAKE_CURRENT_FUNCTION}(...) called without specifying a proto compiler")
        return()
    endif()

    if(NOT DEFINED nanopb_generate_PROTOS)
        message(SEND_ERROR "${CMAKE_CURRENT_FUNCTION}(...) called without any proto source files")
        return()
    endif()

    if(NOT DEFINED nanopb_generate_OUT_VAR)
        message(SEND_ERROR "${CMAKE_CURRENT_FUNCTION}(...) called without specifying an output variable")
        return()
    endif()

    #
    # Resolve nanopb plugin locations
    #

    cmake_path(SET nanopb_plugin_dir
        NORMALIZE
        "${nanopb_SOURCE_DIR}/${NANOPB_PLUGIN_DIR}"
    )

    set(nanopb_plugin_exe)

    if (CMAKE_HOST_WIN32)
        cmake_path(
            APPEND nanopb_plugin_dir
            "${NANOPB_PLUGIN_EXE_WIN32}"
            OUTPUT_VARIABLE nanopb_plugin_exe
        )
    else()
        cmake_path(
            APPEND nanopb_plugin_dir
            "${NANOPB_PLUGIN_EXE_GENERIC}"
            OUTPUT_VARIABLE nanopb_plugin_exe
        )
    endif()

    cmake_path(SET nanopb_plugin_proto_dir
        NORMALIZE
        "${nanopb_plugin_dir}/proto"
    )

    #
    # Argument parsing
    #

    # Default to build tree directory
    if(NOT DEFINED nanopb_generate_PROTOC_OUT_DIR)
        set(nanopb_generate_PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
    endif()

    cmake_path(NORMAL_PATH nanopb_generate_PROTOC_OUT_DIR)

    # Common parent directories for protoc to search for proto files
    set(proto_import_paths)

    foreach(import_dir ${nanopb_generate_IMPORT_DIRS})
        cmake_path(ABSOLUTE_PATH import_dir NORMALIZE)
        list(APPEND proto_import_paths "-I${import_dir}")
    endforeach()

    list(REMOVE_DUPLICATES proto_import_paths)

    # All protoc generated source files
    set(all_generated_srcs)

    foreach(proto_file ${nanopb_generate_PROTOS})
        cmake_path(ABSOLUTE_PATH proto_file NORMALIZE)

        cmake_path(GET proto_file PARENT_PATH proto_file_dir)
        cmake_path(GET proto_file STEM LAST_ONLY proto_file_name)

        if(NANOPB_GENERATE_DEBUG)
            message(DEBUG "Generating for proto file: ${proto_file}")
        endif()

        #
        # Handle relative path imports
        #

        set(proto_file_relative_dir)

        # Get the path of the proto files relative to import directories
        foreach(import_dir ${nanopb_generate_IMPORT_DIRS})
            cmake_path(ABSOLUTE_PATH import_dir NORMALIZE)

            cmake_path(IS_PREFIX import_dir ${proto_file} NORMALIZE is_parent)

            if(is_parent)
                cmake_path(
                    RELATIVE_PATH proto_file_dir
                    BASE_DIRECTORY ${import_dir}
                    OUTPUT_VARIABLE proto_file_relative_dir
                )
                break()
            endif()
        endforeach()

        if(NOT proto_file_relative_dir)
            message(SEND_ERROR "${CMAKE_CURRENT_FUNCTION}(...) could not find a parent import directory for ${proto_file}")
            return()
        endif()

        if(NANOPB_GENERATE_DEBUG)
            message(DEBUG "Found parent import directory: ${proto_file_relative_dir}")
        endif()

        #
        # Search for a nanopb options file for the current proto file
        #

        set(nanopb_options)

        cmake_path(SET proto_options_file
            NORMALIZE
            "${proto_file_dir}/${proto_file_name}${NANOPB_OPTIONS_FILE_EXTENSION}"
        )

        # Pass options file path to nanopb plugin if it exists
        if(EXISTS ${proto_options_file})
            list(APPEND nanopb_options "-I${proto_file_dir}" "-f${proto_options_file}")
        else()
            # Unset here so the generation command doesn't stall waiting on
            # a non-existent file
            unset(proto_options_file)
        endif()

        # Add default options
        list(APPEND nanopb_options ${NANOPB_PLUGIN_DEFAULT_OPTS})

        list(JOIN nanopb_options " " nanopb_options_string)

        set(nanopb_cli_options)

        # See: https://github.com/nanopb/nanopb/blob/c716db13070bfb7de03b33f5a6558528cbf8a249/extra/FindNanopb.cmake#L325
        if(DEFINED NANOPB_PROTOC_OLDER_THAN_3_6_0)
            set(nanopb_cli_options
                "--nanopb_out=${nanopb_options_string}:${nanopb_generate_PROTOC_OUT_DIR}"
            )
        else()
            set(nanopb_cli_options
                "--nanopb_opt=${nanopb_options_string}"
                "--nanopb_out=${nanopb_generate_PROTOC_OUT_DIR}"
            )
        endif()

        #
        # Create output source file paths
        #

        # Generated source files for the current proto file
        set(generated_srcs)

        foreach(extension ${NANOPB_PROTOC_GENERATE_EXTENSIONS})
            set(src_path)
            cmake_path(APPEND src_path
                "${nanopb_generate_PROTOC_OUT_DIR}"
                "${proto_file_relative_dir}"
                "${proto_file_name}${extension}"
            )
            list(APPEND generated_srcs ${src_path})
        endforeach()

        list(APPEND all_generated_srcs ${generated_srcs})

        #
        # Generate sources for the proto file
        #

        add_custom_command(
            OUTPUT ${generated_srcs}
            COMMAND ${nanopb_generate_PROTOC_EXE}
            ARGS "-I${nanopb_plugin_dir}"
                 "-I${nanopb_plugin_proto_dir}"
                 ${proto_import_paths}
                 ${nanopb_cli_options}
                 "--plugin=${NANOPB_PLUGIN_NAME}=${nanopb_plugin_exe}"
                 "${proto_file}"
            DEPENDS ${proto_file}
                    ${proto_options_file}
                    ${nanopb_generate_DEPENDENCIES}
            COMMENT "Running protocol buffer compiler on ${proto_file} with nanopb options: ${nanopb_cli_options}"
            VERBATIM
        )
    endforeach()

    if(NANOPB_GENERATE_DEBUG)
        message(DEBUG "All generated sources: ${all_generated_srcs}")
    endif()

    set_source_files_properties(${all_generated_srcs}
        PROPERTIES GENERATED TRUE
    )

    set(${nanopb_generate_OUT_VAR}
        ${all_generated_srcs}
        PARENT_SCOPE
    )
endfunction()
