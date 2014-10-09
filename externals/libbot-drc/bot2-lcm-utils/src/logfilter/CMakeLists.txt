add_definitions(-std=gnu99)

add_executable(bot-lcm-logfilter
    lcm-logfilter.c)

pods_use_pkg_config_packages(bot-lcm-logfilter 
    lcm glib-2.0)

pods_install_executables(bot-lcm-logfilter)
