add_definitions(-std=gnu99)

# Create an executable program param-tester
add_executable(param-tester param_tester.c)
pods_use_pkg_config_packages(param-tester lcm glib-2.0 bot2-param-client)

# Create an executable program param-tester
add_executable(bot-param-dump param_dump.c)
pods_use_pkg_config_packages(bot-param-dump lcm glib-2.0 bot2-param-client)

# make executable public
pods_install_executables(bot-param-dump)
