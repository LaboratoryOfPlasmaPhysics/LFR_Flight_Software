TEMPLATE = app
# CONFIG += console v8 sim
# CONFIG options = verbose *** boot_messages *** debug_messages *** cpu_usage_report *** stack_report *** vhdl_dev *** debug_tch
CONFIG += console verbose cpu_usage_report boot_messages
CONFIG -= qt

include(./sparc.pri)

# flight software version
SWVERSION=-1-0
DEFINES += SW_VERSION_N1=1 # major
DEFINES += SW_VERSION_N2=0 # minor
DEFINES += SW_VERSION_N3=0 # patch
DEFINES += SW_VERSION_N4=6 # internal

contains( CONFIG, debug_tch ) {
    DEFINES += DEBUG_TCH
}

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

#doxygen.target = doxygen
#doxygen.commands = doxygen ../doc/Doxyfile
#QMAKE_EXTRA_TARGETS += doxygen

TARGET = fsw

INCLUDEPATH += \
    ../src \
    ../header \
    ../src/basic_parameters \
    ../src/avf_prc

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
    ../src/basic_parameters/basic_parameters.c \
    ../src/avf_prc/fsw_processing.c \
    ../src/avf_prc/avf0_prc0.c \
    ../src/avf_prc/avf1_prc1.c \
    ../src/avf_prc/avf2_prc2.c


HEADERS += \
    ../header/wf_handler.h \
    ../header/tc_handler.h \
    ../header/grlib_regs.h \
    ../header/fsw_params.h \
    ../header/fsw_misc.h \
    ../header/fsw_init.h \
    ../header/ccsds_types.h \
    ../header/fsw_params_processing.h \
    ../header/fsw_spacewire.h \
    ../header/tc_load_dump_parameters.h \
    ../header/tm_lfr_tc_exe.h \
    ../header/tc_acceptance.h \
    ../header/fsw_params_nb_bytes.h \
    ../src/basic_parameters/basic_parameters.h \
    ../src/avf_prc/fsw_processing.h \
    ../src/avf_prc/avf0_prc0.h \
    ../src/avf_prc/avf1_prc1.h \
    ../src/avf_prc/avf2_prc2.h

