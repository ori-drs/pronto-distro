cmake_minimum_required(VERSION 2.4.6)

set(POD_NAME bot2-lcmgl)

include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build(JAVA_DEST_DIR ${PROJECT_SOURCE_DIR}/java/src
    PY_DEST_DIR ${PROJECT_SOURCE_DIR}/python/src)

set(ZLIB_LIBRARIES -lz)

add_definitions(-std=gnu99)

add_subdirectory(src/bot_lcmgl_client)
add_subdirectory(src/bot_lcmgl_render)
add_subdirectory(src/lcmgl-viewer)

add_subdirectory(java)
add_subdirectory(python)
