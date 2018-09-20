set(rtems_dir /opt/rtems-4.10/)

set(CMAKE_SYSTEM_NAME rtems)
set(CMAKE_C_COMPILER ${rtems_dir}/bin/sparc-rtems-gcc)
set(CMAKE_CXX_COMPILER ${rtems_dir}/bin/sparc-rtems-g++)
set(CMAKE_LINKER  ${rtems_dir}/bin/sparc-rtems-g++)
SET(CMAKE_EXE_LINKER_FLAGS "-static")
option(fix-b2bst "Activate -mfix-b2bst switch to mitigate \"LEON3FT Stale Cache Entry After Store with Data Tag Parity Error\" errata, GRLIB-TN-0009" ON)

option(Coverage "Enables code coverage" OFF)


set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-O0")


if(fix-b2bst)
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfix-b2bst")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfix-b2bst")
endif()

#if(Coverage)
#    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")
#    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
#endif()


set(CMAKE_C_LINK_EXECUTABLE  "<CMAKE_LINKER> <FLAGS> -Xlinker -Map=<TARGET>.map <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")

include_directories("${rtems_dir}/sparc-rtems/leon3/lib/include")

function (check_b2bst target bin)
    add_custom_command(TARGET ${target}
        POST_BUILD
        COMMAND  ${rtems_dir}/bin/sparc-rtems-objdump -d ${bin}/${target} | ${CMAKE_SOURCE_DIR}/sparc/leon3ft-b2bst-scan.tcl
        )
endfunction()

function (build_srec target bin rev)
    add_custom_command(TARGET ${target}
        POST_BUILD
        COMMAND ${rtems_dir}/bin/sparc-rtems-objcopy -j .data -F srec ${bin}/${target} RpwLfrApp_XXXX_data_rev-${rev}.srec && ${rtems_dir}/bin/sparc-rtems-objcopy -j .text -F srec ${bin}/${target} RpwLfrApp_XXXX_text_rev-${rev}.srec
        )
endfunction()
