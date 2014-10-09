add_definitions(-std=gnu99)

add_executable(bot-lcm-logsplice
    lcm-logsplice.c)

pods_use_pkg_config_packages(bot-lcm-logsplice 
    lcm glib-2.0)

pods_install_executables(bot-lcm-logsplice)

