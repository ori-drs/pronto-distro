find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 eigen3)
if(NOT EIGEN3_FOUND)
    message("eigen3 not found. not building eigen-utils") 
    return()             
endif()

#todo: would be better to just not build the relevant components
pkg_check_modules(BOT_LCMGL bot2-lcmgl-client)
if(NOT BOT_LCMGL_FOUND)
    message("bot2-lcmgl-client not found. not building eigen-utils") 
    return()             
endif()

add_subdirectory(eigen-fftw)


file(GLOB cpp_files *.cpp)
file(GLOB hpp_files *.hpp)
file(GLOB hxx_files *.hxx)

add_library(eigen-utils SHARED
    ${cpp_files})

target_link_libraries(eigen-utils ${OPENGL_LIBRARIES})

set(REQUIRED_LIBS eigen3 bot2-lcmgl-client bot2-core bot2-vis lcmtypes_eigen-utils)

pods_use_pkg_config_packages(eigen-utils ${REQUIRED_LIBS})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(eigen-utils PROPERTIES SOVERSION 1)

# make the header public
pods_install_headers(${hpp_files} ${hxx_files} DESTINATION eigen_utils)

# make the library public
pods_install_libraries(eigen-utils)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(eigen-utils
    CFLAGS
    LIBS -leigen-utils
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
    
    
add_subdirectory(test-eigen)
    
