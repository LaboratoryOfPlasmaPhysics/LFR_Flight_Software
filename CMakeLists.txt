cmake_minimum_required (VERSION 2.6)
project (LFR_FSW)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(LFR_BP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/LFR_basic-parameters/basic_parameters.c)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/sparc")

add_subdirectory(src)
add_subdirectory(timegen)
