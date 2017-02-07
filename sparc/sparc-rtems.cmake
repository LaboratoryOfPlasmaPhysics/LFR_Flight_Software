set(CMAKE_SYSTEM_NAME rtems)

set(CMAKE_C_COMPILER /opt/rtems-4.10/bin/sparc-rtems-gcc)
set(CMAKE_CXX_COMPILER /opt/rtems-4.10/bin/sparc-rtems-g++)
set(CMAKE_LINKER  /opt/rtems-4.10/bin/sparc-rtems-g++)
SET(CMAKE_EXE_LINKER_FLAGS "-static")
set(CMAKE_C_FLAGS_RELEASE "-O3 -mfix-b2bst")
set(CMAKE_C_LINK_EXECUTABLE  "<CMAKE_LINKER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
include_directories("/opt/rtems-4.10/sparc-rtems/leon3/lib/include")
