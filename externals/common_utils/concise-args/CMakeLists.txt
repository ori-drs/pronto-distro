cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME concise-args)
include(cmake/pods.cmake)


# make the header public
#pods_install_headers(ConciseArgs.hpp DESTINATION ConciseArgs)

# make and install the <ConciseArgs> header
configure_file(ConciseArgs.hpp ${INCLUDE_OUTPUT_PATH}/ConciseArgs COPYONLY)
install(FILES ${INCLUDE_OUTPUT_PATH}/ConciseArgs DESTINATION ${INCLUDE_INSTALL_PATH})

#tell cmake to build these subdirectories

add_subdirectory(test)
