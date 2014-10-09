# Create a shared library liblcmgl-utils.so with a single source file
add_library(lcmgl-utils SHARED
    lcmgl_utils.cpp
    LcmglStore.cpp)

# make the header public
# install it to include/lcmgl-utils
pods_install_headers(lcmgl_utils.hpp LcmglStore.hpp DESTINATION lcmgl_utils)

# make the library public
pods_install_libraries(lcmgl-utils)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES bot2-lcmgl-client)
pods_use_pkg_config_packages(lcmgl-utils ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(lcmgl-utils
    CFLAGS
    LIBS -llcmgl-utils
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
