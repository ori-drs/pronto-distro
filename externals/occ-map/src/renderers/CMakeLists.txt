find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building occ-map-renderers")
    return()
endif(NOT BOT2_VIS_FOUND)

add_library(occ-map-renderers SHARED 
    PixelMap_bot_renderer.cpp
    VoxelMap_bot_renderer.cpp)

target_link_libraries (occ-map-renderers
    ${OPENGL_LIBRARIES})

pods_use_pkg_config_packages(occ-map-renderers 
    lcm 
    bot2-vis 
    occ-map)

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(occ-map-renderers PROPERTIES SOVERSION 1)

pods_install_libraries(occ-map-renderers)

pods_install_headers(occ_map_renderers.h DESTINATION occ_map)

pods_install_pkg_config_file(occ-map-renderers
    LIBS -locc-map-renderers
    REQUIRES bot2-vis lcm occ-map
    VERSION 0.0.1)
