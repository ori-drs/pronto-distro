find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
set(GLUT_LIBRARIES -lglut)

pkg_check_modules(BOT2_CLURB bot2-core)
if(NOT BOT2_CLURB_FOUND)
    message("bot2-vis not found.  Not building libbot2 renderer")
    return()
endif(NOT BOT2_CLURB_FOUND)

add_definitions(-Wall -std=gnu99)

add_executable(kinect-rgb-tool rgb_tool.cpp)

target_link_libraries(kinect-rgb-tool
    ${GLUT_LIBRARIES})

pods_use_pkg_config_packages(kinect-rgb-tool
    glib-2.0
    bot2-vis
    bot2-frames
    bot2-param-client 
    lcm
    kinect-utils)

pods_install_executables(kinect-rgb-tool)
