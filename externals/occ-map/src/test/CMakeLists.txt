# Create an executable program vox-map-test
add_executable(vox-map-test VoxelMapTest.cpp)
# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(vox-map-test occ-map)


# Create an executable program pix-map-test
add_executable(pix-map-test PixelMapTest.cpp)
# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(pix-map-test occ-map)
