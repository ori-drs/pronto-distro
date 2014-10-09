add_definitions(-std=gnu99)

# Create an executable program coord-frames-test
add_executable(coord-frames-test coord_frames_test.c)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(coord-frames-test bot2-frames)

# make executable public
#pods_install_executables(coord-frames-test)
