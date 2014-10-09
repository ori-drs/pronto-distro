add_definitions(-DBASE_PATH="${CMAKE_INSTALL_PREFIX}")

add_library(path-util SHARED path_util.c)

# make the shared library public
pods_install_libraries(path-util)
#make the header public
pods_install_headers(path_util.h DESTINATION path_util)

pods_install_pkg_config_file(path-util
    LIBS -lpath-util
    REQUIRES 
    VERSION 0.0.1)
