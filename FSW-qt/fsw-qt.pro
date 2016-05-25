TEMPLATE = app
# CONFIG += console v8 sim
# CONFIG options =
#           verbose
#           boot_messages
#           debug_messages
#           cpu_usage_report
#           stack_report
#           vhdl_dev
#           debug_tch
#           lpp_dpu_destid      /!\ REMOVE BEFORE DELIVERY TO LESIA /!\
#           debug_watchdog
CONFIG += console verbose lpp_dpu_destid cpu_usage_report
CONFIG -= qt

include(./sparc.pri)

# flight software version
SWVERSION=-1-0
DEFINES += SW_VERSION_N1=3 # major
DEFINES += SW_VERSION_N2=1 # minor
DEFINES += SW_VERSION_N3=0 # patch
DEFINES += SW_VERSION_N4=1 # internal

# <GCOV>
#QMAKE_CFLAGS_RELEASE += -fprofile-arcs -ftest-coverage
#LIBS += -lgcov /opt/GCOV/01A/lib/overload.o -lc
# </GCOV>

# <CHANGE BEFORE FLIGHT>
contains( CONFIG, lpp_dpu_destid ) {
    DEFINES += LPP_DPU_DESTID
}
# </CHANGE BEFORE FLIGHT>

contains( CONFIG, debug_tch ) {
    DEFINES += DEBUG_TCH
}
DEFINES += MSB_FIRST_TCH

contains( CONFIG, vhdl_dev ) {
    DEFINES += VHDL_DEV
}

contains( CONFIG, verbose ) {
    DEFINES += PRINT_MESSAGES_ON_CONSOLE
}

contains( CONFIG, debug_messages ) {
    DEFINES += DEBUG_MESSAGES
}

contains( CONFIG, cpu_usage_report ) {
    DEFINES += PRINT_TASK_STATISTICS
}

contains( CONFIG, stack_report ) {
    DEFINES += PRINT_STACK_REPORT
}

contains( CONFIG, boot_messages ) {
    DEFINES += BOOT_MESSAGES
}

contains( CONFIG, debug_watchdog ) {
    DEFINES += DEBUG_WATCHDOG
}

#doxygen.target = doxygen
#doxygen.commands = doxygen ../doc/Doxyfile
#QMAKE_EXTRA_TARGETS += doxygen

TARGET = fsw

INCLUDEPATH += \
    $${PWD}/../src \
    $${PWD}/../header \
    $${PWD}/../header/lfr_common_headers \
    $${PWD}/../header/processing \
    $${PWD}/../LFR_basic-parameters

SOURCES += \
    ../src/wf_handler.c \
    ../src/tc_handler.c \
    ../src/fsw_misc.c \
    ../src/fsw_init.c \
    ../src/fsw_globals.c \
    ../src/fsw_spacewire.c \
    ../src/tc_load_dump_parameters.c \
    ../src/tm_lfr_tc_exe.c \
    ../src/tc_acceptance.c \
    ../src/processing/fsw_processing.c \
    ../src/processing/avf0_prc0.c \
    ../src/processing/avf1_prc1.c \
    ../src/processing/avf2_prc2.c \
    ../src/lfr_cpu_usage_report.c \
    ../LFR_basic-parameters/basic_parameters.c

HEADERS += \
    ../header/wf_handler.h \
    ../header/tc_handler.h \
    ../header/grlib_regs.h \
    ../header/fsw_misc.h \
    ../header/fsw_init.h \
    ../header/fsw_spacewire.h \
    ../header/tc_load_dump_parameters.h \
    ../header/tm_lfr_tc_exe.h \
    ../header/tc_acceptance.h \
    ../header/processing/fsw_processing.h \
    ../header/processing/avf0_prc0.h \
    ../header/processing/avf1_prc1.h \
    ../header/processing/avf2_prc2.h \
    ../header/fsw_params_wf_handler.h \
    ../header/lfr_cpu_usage_report.h \
    ../header/lfr_common_headers/ccsds_types.h \
    ../header/lfr_common_headers/fsw_params.h \
    ../header/lfr_common_headers/fsw_params_nb_bytes.h \
    ../header/lfr_common_headers/fsw_params_processing.h \
    ../header/lfr_common_headers/tm_byte_positions.h \
    ../LFR_basic-parameters/basic_parameters.h \
    ../LFR_basic-parameters/basic_parameters_params.h \
    ../header/GscMemoryLPP.hpp

