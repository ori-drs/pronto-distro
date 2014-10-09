add_definitions(-std=gnu99)

add_library(bot2-param-client SHARED
    param_internal.c param_util.c)
    
set(REQUIRED_LIBS lcm bot2-core glib-2.0 lcmtypes_bot2-param)

pods_use_pkg_config_packages(bot2-param-client ${REQUIRED_LIBS})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(bot2-param-client PROPERTIES SOVERSION 1)

# make the header public
pods_install_headers(param_client.h param_util.h DESTINATION bot_param)

# make the library public
pods_install_libraries(bot2-param-client)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(bot2-param-client
    LIBS -lbot2-param-client
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
