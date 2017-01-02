cmake_minimum_required (VERSION 2.6)
project (LFR_FSW)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(LFR_BP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/LFR_basic-parameters/basic_parameters.c)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/sparc/")

include_directories("header"
	 "header/lfr_common_headers"
	 "header/processing"
         "LFR_basic-parameters"
         "src")

add_subdirectory(src)



option(test "Build all tests." ON) # Makes boolean 'test' available.


if (test)


endif()
