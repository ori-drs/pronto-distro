add_definitions(-std=gnu99)
# Create a shared library liblaser-util.so with a single source file
add_library(laser-util SHARED
    laser_util.c)

# make the header public
# install it to include/laser-util
pods_install_headers(laser_util.h DESTINATION laser_utils)

set(REQUIRED_LIBS glib-2.0 bot2-core bot2-frames bot2-param-client geom-utils)

pods_use_pkg_config_packages(laser-util ${REQUIRED_LIBS})
 
# make the library public
pods_install_libraries(laser-util)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(laser-util
    CFLAGS
    LIBS -llaser-util 
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
