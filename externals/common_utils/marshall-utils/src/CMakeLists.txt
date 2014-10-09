# Create a shared library libmarshall.so with a single source file
add_library(marshall-utils SHARED
    marshall.c)

# make the header public
pods_install_headers(marshall.h DESTINATION marshall_utils)

# make the library public
pods_install_libraries(marshall-utils)

# uncomment these lines to link against another library via pkg-config
#set(REQUIRED_PACKAGES pkg_a pkg_b)
#pods_use_pkg_config_packages(marshall ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(marshall-utils
    CFLAGS
    LIBS -lmarshall-utils
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
