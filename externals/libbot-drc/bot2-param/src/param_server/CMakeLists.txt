add_definitions(-std=gnu99)

# Create an executable program bot-param-server
add_executable(bot-param-server param_server.c lcm_util.c)
pods_use_pkg_config_packages(bot-param-server lcm glib-2.0 bot2-param-client)

# Create an executable program bot-param-tool
add_executable(bot-param-tool param_tool.c)
pods_use_pkg_config_packages(bot-param-tool lcm bot2-param-client)

# make executables public
pods_install_executables(bot-param-server bot-param-tool)

