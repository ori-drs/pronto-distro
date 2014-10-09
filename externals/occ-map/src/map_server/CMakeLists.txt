add_executable(occ-map-pixmap-saver pixmap_saver.cpp)
pods_use_pkg_config_packages(occ-map-pixmap-saver occ-map)   
pods_install_executables(occ-map-pixmap-saver) 

add_executable(occ-map-pixmap-server pixmap_server.cpp)
pods_use_pkg_config_packages(occ-map-pixmap-server occ-map)   
pods_install_executables(occ-map-pixmap-server) 



find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV opencv)
if(NOT OPENCV_FOUND)
    message("opencv not found.  Not building map to image conversions")
    return()
endif(NOT OPENCV_FOUND)


add_executable(occ-map-pixmap-to-image pixmap_to_img.cpp)
pods_use_pkg_config_packages(occ-map-pixmap-to-image occ-map opencv)   
pods_install_executables(occ-map-pixmap-to-image)

add_executable(occ-map-img-to-pixmap img_to_pixmap.cpp)
pods_use_pkg_config_packages(occ-map-img-to-pixmap occ-map opencv)   
pods_install_executables(occ-map-img-to-pixmap)  