add_definitions(-std=gnu99)

file(GLOB c_files *.c)
file(GLOB h_files *.h)

set(libname bot2-lcmgl-client)

add_library(${libname} SHARED ${c_files})

#set(REQUIRED_LIBS ${LIBJPEG_LIBRARIES} ${GLUT_LIBRARIES} ${ZLIB_LIBRARIES})    
target_link_libraries(${libname} ${ZLIB_LIBRARIES})
    
set(REQUIRED_PACKAGES glib-2.0 lcm lcmtypes_bot2-lcmgl)    
pods_use_pkg_config_packages(${libname} ${REQUIRED_PACKAGES})
    

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(${libname} PROPERTIES SOVERSION 1)

# make both the shared library public
pods_install_libraries(${libname})

pods_install_headers(${h_files} DESTINATION bot_lcmgl_client)

pods_install_pkg_config_file(bot2-lcmgl-client
    LIBS -l${libname}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
