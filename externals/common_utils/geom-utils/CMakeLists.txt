cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME geom-utils)

# This lets us use the pods cmake macros
include(cmake/pods.cmake)

add_subdirectory(src)
