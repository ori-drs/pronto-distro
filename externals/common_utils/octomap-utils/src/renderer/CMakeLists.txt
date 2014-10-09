pkg_check_modules(OCTOMAP_UTIL octomap-util)
if(NOT OCTOMAP_UTIL_FOUND)
    message("octomap_util not found. Not building octomap libbot2 renderer")
    return()
endif()

pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building octomap libbot2 renderer")
    return()
endif()

# Create a shared library liboctomap-renderer.so with a single source file
add_library(octomap-renderer SHARED
    renderer_octomap.cpp)

# make the header public
# install it to include/octomap-renderer
pods_install_headers(renderer_octomap.h DESTINATION octomap_utils)

target_link_libraries(octomap-renderer ${OPENGL_LIBRARIES})

set(REQUIRED_PACKAGES bot2-vis octomap-util)
pods_use_pkg_config_packages(octomap-renderer ${REQUIRED_PACKAGES})


# make the library public
pods_install_libraries(octomap-renderer)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(octomap-renderer
    CFLAGS
    LIBS -loctomap-renderer
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
