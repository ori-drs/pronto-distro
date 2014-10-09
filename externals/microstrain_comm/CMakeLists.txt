cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME microstrain_comm)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# create an executable, and make it public
add_executable(microstrain_comm src/microstrain_comm.cpp)
pods_use_pkg_config_packages(microstrain_comm bot2-core lcmtypes_microstrain_comm bot2-param-client)
pods_install_executables(microstrain_comm)


# pass through microstrain_comm quaternion as a bot_core_pose_t (for simple visualization)
add_executable(microstrain_pose src/microstrain_pose.cpp)
pods_use_pkg_config_packages(microstrain_pose lcmtypes_microstrain_comm)
pods_install_executables(microstrain_pose)
