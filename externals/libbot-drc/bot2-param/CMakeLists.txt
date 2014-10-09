cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME bot2-param)

include(cmake/pods.cmake)
pods_config_search_paths()

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/param_client)
add_subdirectory(src/param_server)
add_subdirectory(src/param_tester)
