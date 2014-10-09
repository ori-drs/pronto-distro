add_definitions(-std=gnu99)

file(GLOB c_files *.c)
file(GLOB h_files *.h)
add_library(geom-utils SHARED ${c_files})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(geom-utils PROPERTIES SOVERSION 1)
target_link_libraries(geom-utils m)
pods_use_pkg_config_packages(geom-utils 
    glib-2.0)

# make the shared library public
pods_install_libraries(geom-utils)
#make the header public
pods_install_headers(${h_files} DESTINATION geom_utils)

pods_install_pkg_config_file(geom-utils
    LIBS -lgeom-utils
    REQUIRES glib-2.0
    VERSION 0.0.1)
