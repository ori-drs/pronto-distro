find_package(PkgConfig REQUIRED)
pkg_check_modules(OCC_MAP occ-map)
if(OCC_MAP_FOUND)
	add_library(laser-sim2d SHARED
	    LaserSim2D.cpp)
	pods_install_headers(LaserSim2D.hpp DESTINATION laser_utils)
	
	set(REQUIRED_LIBS bot2-core geom-utils occ-map)
	pods_use_pkg_config_packages(laser-sim2d ${REQUIRED_LIBS})
	 
	# make the library public
	pods_install_libraries(laser-sim2d)
	
	# create a pkg-config file for the library, to make it easier for other
	# software to use.
	pods_install_pkg_config_file(laser-sim2d
	    CFLAGS
	    LIBS -llaser-sim2d 
	    REQUIRES ${REQUIRED_LIBS}
	    VERSION 0.0.1)
	
	
	add_executable(laser-simulator2d simulator2D.cpp)
	pods_use_pkg_config_packages(laser-simulator2d bot2-core bot2-param-client bot2-frames laser-sim2d)   
	pods_install_executables(laser-simulator2d) 
	
else()
	  	message("occ-map not found. not building 2D laser-simulator")
endif()


pkg_check_modules(OCTOMAP octomap)
if(OCTOMAP_FOUND)
	add_library(laser-sim3d SHARED
	    LaserSim3D.cpp)
	pods_install_headers(LaserSim3D.hpp DESTINATION laser_utils)
	
	set(REQUIRED_LIBS bot2-core geom-utils octomap laser-util)
	pods_use_pkg_config_packages(laser-sim3d ${REQUIRED_LIBS})
	 
	# make the library public
	pods_install_libraries(laser-sim3d)
	
	# create a pkg-config file for the library, to make it easier for other
	# software to use.
	pods_install_pkg_config_file(laser-sim3d
	    CFLAGS
	    LIBS -llaser-sim3d 
	    REQUIRES ${REQUIRED_LIBS}
	    VERSION 0.0.1)
	
	
	add_executable(laser-simulator3d simulator3D.cpp)
	pods_use_pkg_config_packages(laser-simulator3d bot2-core bot2-param-client bot2-frames laser-sim3d)   
	pods_install_executables(laser-simulator3d) 
	
else()
	  	message("octomap not found. not building 3D laser-simulator")
endif()