TEMPLATE = app
# CONFIG += console v8 sim
# CONFIG options = verbose *** boot_messages *** debug_messages *** cpu_usage_report *** stack_report *** vhdl_dev *** debug_tch
CONFIG += console verbose boot_messages debug_messages
CONFIG -= qt

include(./sparc.pri)

# flight software version
SWVERSION=-1-0
DEFINES += SW_VERSION_N1=0 # major
DEFINES += SW_VERSION_N2=0 # minor
DEFINES += SW_VERSION_N3=0 # patch
DEFINES += SW_VERSION_N4=1 # internal

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

TARGET = timegen

INCLUDEPATH += \
    src \
    header \
    ../src \
    ../header

SOURCES += \
    src/timegen_init.c \
    src/timegen_tc_handler.c \
    src/timegen_misc.c \
    src/timegen_spacewire.c \
    ../src/fsw_misc.c \
    ../src/fsw_globals.c \
    ../src/tm_lfr_tc_exe.c \
    ../src/tc_acceptance.c

HEADERS += \
    ../header/grlib_regs.h \
    ../header/fsw_params.h \
    ../header/fsw_misc.h \
    ../header/ccsds_types.h \
    ../header/fsw_params_processing.h \
    ../header/tm_lfr_tc_exe.h \
    ../header/tc_acceptance.h \
    ../header/fsw_params_nb_bytes.h \
    ../header/TC_types.h \
    header/timegen_init.h \
    header/timegen_tc_handler.h \
    header/timegen_misc.h \
    header/timegen_spacewire.h
