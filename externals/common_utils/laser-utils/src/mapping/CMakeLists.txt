find_package(PkgConfig REQUIRED)
pkg_check_modules(OCC_MAP occ-map)
if(OCC_MAP_FOUND)
    add_executable(laser-voxmapper laser_voxmapper.cpp)
    pods_use_pkg_config_packages(laser-voxmapper 
                    bot2-core bot2-param-client bot2-frames
                    occ-map laser-util)   
    pods_install_executables(laser-voxmapper) 
    
    
    add_executable(laser-pixmapper laser_pixmapper.cpp)
    pods_use_pkg_config_packages(laser-pixmapper 
                    bot2-core bot2-param-client bot2-frames
                    occ-map laser-util)   
    pods_install_executables(laser-pixmapper) 
    
else()
    message("occ-map not found. not building laser-voxmapper")              
endif()

pkg_check_modules(OCTOMAP octomap)
if(OCTOMAP_FOUND) 
    add_executable(laser-octomapper laser_octomapper.cpp)
    pods_use_pkg_config_packages(laser-octomapper 
                    bot2-core bot2-param-client bot2-frames
                    octomap laser-util lcmtypes_laser-utils)
                   
    pods_install_executables(laser-octomapper)
else()
    message("octomap not found. not building laser-octomapper")
endif()
