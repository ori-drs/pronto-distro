add_definitions(-std=gnu99)

file(GLOB c_files *.c)
file(GLOB h_files *.h)

add_library(bot2-lcmgl-renderer SHARED ${c_files})

set(REQUIRED_LIBS bot2-vis lcmtypes_bot2-lcmgl gl)
pods_use_pkg_config_packages(bot2-lcmgl-renderer ${REQUIRED_LIBS})
    

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(bot2-lcmgl-renderer PROPERTIES SOVERSION 1)

pods_install_libraries(bot2-lcmgl-renderer)

pods_install_headers(${h_files} DESTINATION bot_lcmgl_render)

pods_install_pkg_config_file(bot2-lcmgl-renderer
    LIBS -lbot2-lcmgl-renderer
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
