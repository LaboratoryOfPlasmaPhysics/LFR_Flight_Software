cmake_minimum_required (VERSION 2.6)
project (timegen)

include(sparc-rtems)

include_directories("./src"
	 "./header"
	 "./header/processing"
         "./src/LFR_basic-parameters")

set(SOURCES  src/tc_handler.c
    src/fsw_misc.c
    src/fsw_init.c
    src/fsw_globals.c
    src/fsw_spacewire.c
    src/tc_acceptance.c
    ../LFR_basic-parameters/basic_parameters.c)


option(timegen_verbose "Enable verbose Timegen" ON)
option(timegen_boot_messages "Enable Timegen boot messages" ON)
option(timegen_debug_messages "Enable Timegen debug messages" ON)
option(timegen_cpu_usage_report "Enable Timegen cpu usage report" OFF)
option(timegen_stack_report "Enable Timegen stack report" OFF)
option(timegen_vhdl_dev "?" OFF)
option(timegen_lpp_dpu_destid "Set to debug at LPP" ON)
option(timegen_debug_watchdog "Enable debug watchdog" OFF)
option(timegen_debug_tch "?" OFF)

set(TIMEGEN_SW_VERSION_N1 "3" CACHE STRING  "Choose N1 FSW Version." FORCE)
set(TIMEGEN_SW_VERSION_N2 "1" CACHE STRING  "Choose N2 FSW Version." FORCE)
set(TIMEGEN_SW_VERSION_N3 "0" CACHE STRING  "Choose N3 FSW Version." FORCE)
set(TIMEGEN_SW_VERSION_N4 "4" CACHE STRING  "Choose N4 FSW Version." FORCE)


if (timegen_verbose)
	add_definitions(-DPRINT_MESSAGES_ON_CONSOLE)
endif()
if (timegen_boot_messages)
	add_definitions(-DBOOT_MESSAGES)
endif()
if (timegen_debug_messages)
	add_definitions(-DDEBUG_MESSAGES)
endif()
if (timegen_cpu_usage_report)
	add_definitions(-DPRINT_TASK_STATISTICS)
endif()
if (timegen_stack_report)
	add_definitions(-DPRINT_STACK_REPORT)
endif()
if (timegen_vhdl_dev)
	add_definitions(-DVHDL_DEV)
endif()
if (timegen_lpp_dpu_destid)
	add_definitions(-DLPP_DPU_DESTID)
endif()
if (timegen_debug_watchdog)
	add_definitions(-DDEBUG_WATCHDOG)
endif()
if (timegen_debug_tch)
	add_definitions(-DDEBUG_TCH)
endif()

add_definitions(-DMSB_FIRST_TCH)

add_definitions(-DSWVERSION=-1-0)
add_definitions(-DSW_VERSION_N1=${TIMEGEN_SW_VERSION_N1})
add_definitions(-DSW_VERSION_N2=${TIMEGEN_SW_VERSION_N2})
add_definitions(-DSW_VERSION_N3=${TIMEGEN_SW_VERSION_N3})
add_definitions(-DSW_VERSION_N4=${TIMEGEN_SW_VERSION_N4})

add_executable(timegen ${SOURCES})
