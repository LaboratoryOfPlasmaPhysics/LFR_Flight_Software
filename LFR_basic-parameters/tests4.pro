TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += DEBUG_TCH
#DEFINES += MSB_FIRST_TCH    # SPARC convention
DEFINES += LSB_FIRST_TCH    # PC convention

SOURCES += main.c \
    basic_parameters.c \
    file_utilities.c

HEADERS += \
    basic_parameters.h \
    basic_parameters_params.h \
    basic_parameters_utilities.h \
    file_utilities.h


