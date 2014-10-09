file(GLOB c_files *.c)
file(GLOB h_files *.h)

# Keep the GLM library out of the public API
list(REMOVE_ITEM h_files glm.h)

add_library(bot2-vis SHARED ${c_files})

set(REQUIRED_LIBS ${LIBJPEG_LIBRARIES} ${GLUT_LIBRARIES} ${ZLIB_LIBRARIES})    
if (APPLE)  
# note: the following lines will now work on every platform (and
# really should be included), but they depend on a fix to pods.cmake,
# which has to be copied around to all dependent pods.  For this reason,
# I've unfortunately restricted the fix to APPLE for the short term.  - Russ T
  find_package( X11 REQUIRED )
  include_directories ( ${X11_INCLUDE_DIR} )
  set(REQUIRED_LIBS ${REQUIRED_LIBS} ${X11_LIBRARIES})
endif(APPLE)

target_link_libraries(bot2-vis ${REQUIRED_LIBS})

set(REQUIRED_PACAKGES glib-2.0 gtk+-2.0 gdk-pixbuf-2.0 lcm bot2-core libpng gl glu)
pods_use_pkg_config_packages(bot2-vis ${REQUIRED_PACAKGES})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(bot2-vis PROPERTIES SOVERSION 1)

pods_install_libraries(bot2-vis)

pods_install_headers(${h_files} DESTINATION bot_vis)

pods_install_pkg_config_file(${PROJECT_NAME}
    LIBS -lbot2-vis ${REQUIRED_LIBS}
    REQUIRES ${REQUIRED_PACAKGES}
    VERSION 0.0.1)
