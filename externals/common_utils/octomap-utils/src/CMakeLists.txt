pkg_check_modules(OCC_MAP occ-map)
if(NOT OCC_MAP_FOUND)
    message("occ-map not found.  Not building octomap-util")
    return()
endif()


# Create a shared library liboctomap-util.so with a single source file
add_library(octomap-util SHARED
    octomap_util.cpp)

set(REQUIRED_PACKAGES occ-map octomap laser-util lcmtypes_octomap-utils)
pods_use_pkg_config_packages(octomap-util ${REQUIRED_PACKAGES})


# make the header public
# install it to include/octomap-util
pods_install_headers(octomap_util.hpp DESTINATION octomap_utils)

# make the library public
pods_install_libraries(octomap-util)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(octomap-util
    CFLAGS
    LIBS -loctomap-util
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

    
add_executable(octomap2voxelmap octomap2voxelmap.cpp)
pods_use_pkg_config_packages(octomap2voxelmap octomap-util)
pods_install_executables(octomap2voxelmap)

add_executable(octomap-blurrer octomap-blurrer.cpp)
pods_use_pkg_config_packages(octomap-blurrer octomap-util)
pods_install_executables(octomap-blurrer)

add_executable(octomap-dilator octomap-dilator.cpp)
pods_use_pkg_config_packages(octomap-dilator octomap-util)
pods_install_executables(octomap-dilator)

add_executable(octomap-server octomap-server.cpp)
pods_use_pkg_config_packages(octomap-server octomap-util)
pods_install_executables(octomap-server)
