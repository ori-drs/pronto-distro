cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME eigen-utils)
include(cmake/pods.cmake)
include(cmake/lcmtypes.cmake)
include(cmake/matlab_pods.cmake)
lcmtypes_build()

# Necessary to build on OSX
find_package(OpenGL REQUIRED)

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(matlab)
pods_install_matlab_path(eigen_utils)