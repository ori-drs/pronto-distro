set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


add_definitions(-Wall )
#add_definitions(-Wall -std=gnu99) # .... compiler warning

##############################################
# Basic Test Program
add_executable (se-icpcuda icpcuda-app.cpp)
target_link_libraries(se-icpcuda boost_system)
pods_use_pkg_config_packages(se-icpcuda eigen3
  lcm
  lcmtypes_bot2-core
  lidar-odom
  ${FK_PACKAGES})
pods_install_executables(se-icpcuda)



