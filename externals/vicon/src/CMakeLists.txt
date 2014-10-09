if( CMAKE_SYSTEM_PROCESSOR STREQUAL "x64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64" )
  message("Linking against 64-bit Vicon library")
  set(vicon_lib_name "ViconDataStreamSDK_CPP_x64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i686")
  message("Linking against 32-bit Vicon library")
  set(vicon_lib_name "ViconDataStreamSDK_CPP_x86")
else()
  message("\n--- Don't have a precompiled ViconDataStreamSDK for ${CMAKE_SYSTEM_PROCESSOR}")
  message("--- Not building the vicon driver\n")
  return()
endif()

# Create an executable program vicon-client
add_executable(vicon-client vicon_client.cpp)
target_link_libraries(vicon-client -l${vicon_lib_name})
pods_use_pkg_config_packages(vicon-client lcm lcmtypes_vicon)

# make executable public
pods_install_executables(vicon-client)


#copy over/install the vicon sdk shared object
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/lib${vicon_lib_name}.so ${CMAKE_BINARY_DIR}/lib/lib${vicon_lib_name}.so COPYONLY)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib${vicon_lib_name}.so 
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

   
# Create an executable program vicon-client
add_executable(vicon-frames-updater vicon_frames_updater.cpp)
pods_use_pkg_config_packages(vicon-frames-updater lcm bot2-frames lcmtypes_vicon)

# make executable public
pods_install_executables(vicon-frames-updater)

