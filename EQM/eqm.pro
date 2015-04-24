TEMPLATE = app
# CONFIG += console v8 sim
# CONFIG options = verbose *** boot_messages *** debug_messages *** cpu_usage_report *** stack_report *** vhdl_dev *** debug_tch
# lpp_dpu_destid
CONFIG += console verbose lpp_dpu_destid cpu_usage_report
CONFIG -= qt

include(./sparc.pri)

# eqm debug software version
SWVERSION=-1-0
DEFINES += SW_VERSION_N1=0 # major
DEFINES += SW_VERSION_N2=0 # minor
DEFINES += SW_VERSION_N3=0 # patch
DEFINES += SW_VERSION_N4=0 # internal

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

#doxygen.target = doxygen
#doxygen.commands = doxygen ../doc/Doxyfile
#QMAKE_EXTRA_TARGETS += doxygen

TARGET = eqm

INCLUDEPATH += ./header \
    ../header/lfr_common_headers

SOURCES += \
    src/main.c \
    src/grspw.c

HEADERS += \
    ../header/lfr_common_headers/fsw_params.h \
    header/grspw.h



