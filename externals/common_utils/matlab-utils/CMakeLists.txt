cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME matlab-utils)
include(cmake/pods.cmake)
include(cmake/matlab_pods.cmake)

pods_configure_matlab_paths()

pods_install_matlab_path(matlab_utils)
