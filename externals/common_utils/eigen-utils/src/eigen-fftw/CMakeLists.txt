find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 fftw3)
if(NOT FFTW3_FOUND)
    message("fftw not found. not building eigen-utils-fftw") 
    return()             
endif()

file(GLOB cpp_files *.cpp)
file(GLOB hpp_files *.hpp)
file(GLOB hxx_files *.hxx)

add_library(eigen-utils-fftw SHARED
    ${cpp_files})

set(REQUIRED_LIBS eigen3 fftw3 fftw3f)

pods_use_pkg_config_packages(eigen-utils-fftw ${REQUIRED_LIBS})

# make the header public
pods_install_headers(${hpp_files} ${hxx_files} DESTINATION eigen_utils)

# make the library public
pods_install_libraries(eigen-utils-fftw)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(eigen-utils-fftw
    CFLAGS
    LIBS -leigen-utils-fftw
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)


#build the test executable
add_executable(eigen-test-fftw test_eigen_fftw.cpp)
pods_use_pkg_config_packages(eigen-test-fftw eigen-utils-fftw)
