file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/pods_path.m 
                               "function path = pods_path()\n"
                               "    path = '${CMAKE_INSTALL_PREFIX}/';\n"
                               "end"
                               )