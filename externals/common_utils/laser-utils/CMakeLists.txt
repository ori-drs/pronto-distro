cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME laser-utils)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

#check for bot_frames since it requires bot_core and bot_param
find_package(PkgConfig REQUIRED)
pkg_check_modules(BOT_FRAMES bot2-frames)
if(NOT BOT_FRAMES_FOUND)
    message("bot-frames not found. not building laser-utils") 
    return()             
endif()

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(src/renderer)
add_subdirectory(src/mapping)
add_subdirectory(src/simulator)
