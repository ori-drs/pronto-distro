cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME bot2-vis)

include_directories(${PROJECT_SOURCE_DIR}/src)

include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)

set(GLUT_LIBRARIES -lglut)

set(ZLIB_LIBRARIES -lz)

set(LIBJPEG_CFLAGS "")
set(LIBJPEG_LIBRARIES -ljpeg)

add_definitions(-std=gnu99)

add_subdirectory(src/bot_vis)
add_subdirectory(src/testers)
add_subdirectory(src/rwx-viewer)
add_subdirectory(src/wavefront-viewer)

# bot-ppmsgz script
install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/bot-ppmsgz DESTINATION bin)
