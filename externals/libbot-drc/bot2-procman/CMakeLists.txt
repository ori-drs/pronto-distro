cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME bot2-procman)

include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build(PY_DEST_DIR ${PROJECT_SOURCE_DIR}/python/src)


add_subdirectory(src/deputy)
add_subdirectory(python)
