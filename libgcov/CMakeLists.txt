cmake_minimum_required(VERSION 3.6)
project(libgcov C)
include(sparc-rtems)
include(cppcheck)

set(LIB_GCOV_SOURCES
    gcov-io.c
    gcov-io.h
    gcov-iov.h
    libgcov.c
)
if(Coverage)
#    add_definitions(-DGCOV_USE_EXIT)
    add_definitions(-DGCOV_ENABLED)
endif()
add_library(gcov STATIC ${LIB_GCOV_SOURCES})

add_custom_target(gcovr
    COMMAND gcovr --exclude='.*gcov.*' --gcov-executable=${rtems_dir}/bin/sparc-rtems-gcov  --object-directory ${CMAKE_BINARY_DIR}  -r ${CMAKE_SOURCE_DIR}  --html  --html-details -o ${CMAKE_CURRENT_BINARY_DIR}/gcov.html && xdg-open ${CMAKE_CURRENT_BINARY_DIR}/gcov.html
    )
