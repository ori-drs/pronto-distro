# Create an executable program marshall-test
add_executable(marshall-test marshall-test.c)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(marshall-test marshall-utils)

# make executable public
pods_install_executables(marshall-test)
