cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME octomap-utils)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()


find_package(PkgConfig REQUIRED)

find_package(OpenGL REQUIRED)

list(APPEND OPENGL_LIBRARIES GL)

pkg_check_modules(OCTOMAP octomap)
if(NOT OCTOMAP_FOUND)
  message("\n\n---- noctomap not found. Not building octomap-utils\n")
  return()
endif()

find_package(PkgConfig REQUIRED)

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(src/renderer)
