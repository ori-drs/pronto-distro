add_definitions(-std=gnu99)

add_executable(bot-lcm-who
    lcm-who.c
    signal_pipe.c)

pods_use_pkg_config_packages(bot-lcm-who 
    lcm glib-2.0)

pods_install_executables(bot-lcm-who)
