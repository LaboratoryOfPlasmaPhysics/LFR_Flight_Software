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


option(verbose "Enable verbose LFR" ON)
option(boot_messages "Enable LFR boot messages" ON)
option(debug_messages "Enable LFR debug messages" ON)
option(cpu_usage_report "Enable LFR cpu usage report" OFF)
option(stack_report "Enable LFR stack report" OFF)
option(vhdl_dev "?" OFF)
option(lpp_dpu_destid "Set to debug at LPP" ON)
option(debug_watchdog "Enable debug watchdog" OFF)
option(debug_tch "?" OFF)


if (verbose)
	add_definitions(-DPRINT_MESSAGES_ON_CONSOLE)
endif()
if (boot_messages)
	add_definitions(-DBOOT_MESSAGES)
endif()
if (debug_messages)
	add_definitions(-DDEBUG_MESSAGES)
endif()
if (cpu_usage_report)
	add_definitions(-DPRINT_TASK_STATISTICS)
endif()
if (stack_report)
	add_definitions(-DPRINT_STACK_REPORT)
endif()
if (vhdl_dev)
	add_definitions(-DVHDL_DEV)
endif()
if (lpp_dpu_destid)
	add_definitions(-DLPP_DPU_DESTID)
endif()
if (debug_watchdog)
	add_definitions(-DDEBUG_WATCHDOG)
endif()
if (debug_tch)
	add_definitions(-DDEBUG_TCH)
endif()

add_definitions(-DMSB_FIRST_TCH)

add_definitions(-DSWVERSION=-1-0)
add_definitions(-DSW_VERSION_N1=3)
add_definitions(-DSW_VERSION_N2=1)
add_definitions(-DSW_VERSION_N3=0)
add_definitions(-DSW_VERSION_N4=4)

add_executable(timegen ${SOURCES})
