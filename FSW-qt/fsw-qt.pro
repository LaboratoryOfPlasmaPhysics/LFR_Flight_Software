TEMPLATE = app
# CONFIG += console v8 sim
# CONFIG options = verbose *** cpu_usage_report *** gsa
CONFIG += console verbose
CONFIG -= qt

include(./sparc.pri)

# flight software version
SWVERSION=-0-13
DEFINES += SW_VERSION_N1=0
DEFINES += SW_VERSION_N2=0
DEFINES += SW_VERSION_N3=0
DEFINES += SW_VERSION_N4=13

contains( CONFIG, verbose ) {
    DEFINES += PRINT_MESSAGES_ON_CONSOLE
}

contains( CONFIG, cpu_usage_report ) {
    DEFINES += PRINT_TASK_STATISTICS
}

TARGET = fsw
contains( CONFIG, gsa ) {
    DEFINES += GSA
    TARGET = fsw-gsa
}

INCLUDEPATH += \
    ../src \
    ../header

SOURCES += \
    ../src/wf_handler.c \
    ../src/tc_handler.c \
    ../src/fsw_processing.c \
    ../src/fsw_misc.c \
    ../src/fsw_init.c \
    ../src/fsw_globals.c \
    ../src/fsw_spacewire.c

HEADERS += \
    ../header/wf_handler.h \
    ../header/tc_handler.h \
    ../header/grlib_regs.h \
    ../header/fsw_processing.h \
    ../header/fsw_params.h \
    ../header/fsw_misc.h \
    ../header/fsw_init.h \
    ../header/ccsds_types.h \
    ../header/fsw_params_processing.h \
    ../header/fsw_spacewire.h

