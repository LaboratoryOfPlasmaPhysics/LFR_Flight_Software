cmake_minimum_required (VERSION 3.5)
project (fsw)

include(sparc-rtems)
include(cppcheck)

include_directories("../header"
    "../header/lfr_common_headers"
    "../header/processing"
    "../LFR_basic-parameters"
    "../src")

set(SOURCES    wf_handler.c
    tc_handler.c
    fsw_misc.c
    fsw_init.c
    fsw_globals.c
    fsw_spacewire.c
    tc_load_dump_parameters.c
    tm_lfr_tc_exe.c
    tc_acceptance.c
    mitigations/PAS_filtering.c
    mitigations/reaction_wheel_filtering.c
    processing/fsw_processing.c
    processing/avf0_prc0.c
    processing/avf1_prc1.c
    processing/avf2_prc2.c
    processing/ASM/spectralmatrices.c
    lfr_cpu_usage_report.c
    ${LFR_BP_SRC}
    ../header/mitigations/PAS_filtering.h
    ../header/mitigations/reaction_wheel_filtering.h
    ../header/wf_handler.h
    ../header/tc_handler.h
    ../header/grlib_regs.h
    ../header/fsw_misc.h
    ../header/fsw_init.h
    ../header/fsw_spacewire.h
    ../header/tc_load_dump_parameters.h
    ../header/tm_lfr_tc_exe.h
    ../header/tc_acceptance.h
    ../header/processing/fsw_processing.h
    ../header/processing/avf0_prc0.h
    ../header/processing/avf1_prc1.h
    ../header/processing/avf2_prc2.h
    ../header/processing/ASM/spectralmatrices.h
    ../header/fsw_params_wf_handler.h
    ../header/lfr_cpu_usage_report.h
    ../header/lfr_common_headers/ccsds_types.h
    ../header/lfr_common_headers/fsw_params.h
    ../header/lfr_common_headers/fsw_params_nb_bytes.h
    ../header/lfr_common_headers/fsw_params_processing.h
    ../header/lfr_common_headers/tm_byte_positions.h
    ../LFR_basic-parameters/basic_parameters.h
    ../LFR_basic-parameters/basic_parameters_params.h
    ../header/GscMemoryLPP.hpp
    )


option(FSW_verbose "Enable verbose LFR" OFF)
option(FSW_boot_messages "Enable LFR boot messages" OFF)
option(FSW_debug_messages "Enable LFR debug messages" OFF)
option(FSW_cpu_usage_report "Enable LFR cpu usage report" OFF)
option(FSW_stack_report "Enable LFR stack report" OFF)
option(FSW_vhdl_dev "?" OFF)
option(FSW_lpp_dpu_destid "Set to debug at LPP" OFF)
option(FSW_debug_watchdog "Enable debug watchdog" OFF)
option(FSW_debug_tch "?" OFF)
option(FSW_Instrument_Scrubbing "Enable scrubbing counter" OFF)
option(FSW_Enable_Dead_Code "Enable dead code compilation, this is used to hide by default unused code." OFF)

set(SW_VERSION_N1 "3" CACHE STRING  "Choose N1 FSW Version." FORCE)
set(SW_VERSION_N2 "2" CACHE STRING  "Choose N2 FSW Version." FORCE)
set(SW_VERSION_N3 "0" CACHE STRING  "Choose N3 FSW Version." FORCE)
set(SW_VERSION_N4 "24" CACHE STRING  "Choose N4 FSW Version." FORCE)

if(FSW_verbose)
    add_definitions(-DPRINT_MESSAGES_ON_CONSOLE)
endif()
if(FSW_boot_messages)
    add_definitions(-DBOOT_MESSAGES)
endif()
if(FSW_debug_messages)
    add_definitions(-DDEBUG_MESSAGES)
endif()
if(FSW_cpu_usage_report)
    add_definitions(-DPRINT_TASK_STATISTICS)
endif()
if(FSW_stack_report)
    add_definitions(-DPRINT_STACK_REPORT)
endif()
if(FSW_vhdl_dev)
    add_definitions(-DVHDL_DEV)
endif()
if(FSW_lpp_dpu_destid)
    add_definitions(-DLPP_DPU_DESTID)
endif()
if(FSW_debug_watchdog)
    add_definitions(-DDEBUG_WATCHDOG)
endif()
if(FSW_debug_tch)
    add_definitions(-DDEBUG_TCH)
endif()

if(FSW_Enable_Dead_Code)
    add_definitions(-DENABLE_DEAD_CODE)
endif()




add_definitions(-DMSB_FIRST_TCH)

add_definitions(-DSWVERSION=-1-0)
add_definitions(-DSW_VERSION_N1=${SW_VERSION_N1})
add_definitions(-DSW_VERSION_N2=${SW_VERSION_N2})
add_definitions(-DSW_VERSION_N3=${SW_VERSION_N3})
add_definitions(-DSW_VERSION_N4=${SW_VERSION_N4})

add_executable(fsw ${SOURCES})

if(FSW_Instrument_Scrubbing)
    add_definitions(-DENABLE_SCRUBBING_COUNTER)
endif()

if(Coverage)
    target_link_libraries(fsw gcov)
    SET_TARGET_PROPERTIES(fsw PROPERTIES COMPILE_FLAGS  "-fprofile-arcs -ftest-coverage")
endif()


if(fix-b2bst)
    check_b2bst(fsw ${CMAKE_CURRENT_BINARY_DIR})
endif()

if(NOT FSW_lpp_dpu_destid)
    build_srec(fsw ${CMAKE_CURRENT_BINARY_DIR} "${SW_VERSION_N1}-${SW_VERSION_N2}-${SW_VERSION_N3}-${SW_VERSION_N4}")
endif()


#add_test_cppcheck(fsw STYLE UNUSED_FUNCTIONS POSSIBLE_ERROR MISSING_INCLUDE)

