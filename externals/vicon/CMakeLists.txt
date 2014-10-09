cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME vicon)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

#tell cmake to build these subdirectories
add_subdirectory(src)
