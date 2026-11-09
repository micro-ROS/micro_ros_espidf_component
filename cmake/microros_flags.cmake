# Copyright 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

# IDF 5 uses ordinary flag strings; IDF 6 also uses GCC response files.
# Snapshot their contents so changing flags at the same path invalidates the
# generated micro-ROS toolchain, without rebuilding on timestamp-only changes.
function(microros_expand_flags output)
    set(result "")
    foreach(arg IN LISTS ARGN)
        if(arg MATCHES "^@(.+)$")
            set(response_file "${CMAKE_MATCH_1}")
            file(READ "${response_file}" contents)
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${response_file}")
            separate_arguments(nested UNIX_COMMAND "${contents}")
            microros_expand_flags(expanded ${nested})
            list(APPEND result ${expanded})
        elseif(arg MATCHES "^SHELL:(.*)$")
            separate_arguments(expanded UNIX_COMMAND "${CMAKE_MATCH_1}")
            list(APPEND result ${expanded})
        elseif(arg MATCHES "\\$<")
            message(FATAL_ERROR
                "micro-ROS is built during CMake configuration and cannot evaluate "
                "the IDF compile option '${arg}'.")
        else()
            list(APPEND result "${arg}")
        endif()
    endforeach()
    set(${output} "${result}" PARENT_SCOPE)
endfunction()

function(microros_get_flags language output)
    separate_arguments(toolchain_flags UNIX_COMMAND "${CMAKE_${language}_FLAGS}")
    idf_build_get_property(common_flags COMPILE_OPTIONS)
    idf_build_get_property(language_flags ${language}_COMPILE_OPTIONS)
    idf_build_get_property(definitions COMPILE_DEFINITIONS)
    microros_expand_flags(flags ${toolchain_flags} ${common_flags} ${language_flags})

    # ROS packages own their warning policy and language standards. Keep IDF's
    # architecture, ABI, libc, workaround, optimization and instrumentation flags.
    # Keep -Wa, / -Wp, / -Wl, driver forwarding options; they are not warnings.
    list(FILTER flags EXCLUDE REGEX "^-W([^alp]|[alp][^,]|[alp]$)")
    list(FILTER flags EXCLUDE REGEX "^-std=")
    foreach(definition IN LISTS definitions)
        if(NOT definition MATCHES "^-D")
            set(definition "-D${definition}")
        endif()
        list(APPEND flags "${definition}")
    endforeach()
    microros_expand_flags(flags ${flags})

    # Preserve argument boundaries, including macro values and paths with spaces,
    # when CMake passes this string to the child compiler command line.
    set(quoted_flags "")
    foreach(flag IN LISTS flags)
        string(REPLACE "\\" "\\\\" flag "${flag}")
        string(REPLACE "\"" "\\\"" flag "${flag}")
        string(APPEND quoted_flags " \"${flag}\"")
    endforeach()
    set(${output} "${quoted_flags}" PARENT_SCOPE)
endfunction()
