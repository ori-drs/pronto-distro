add_definitions(-std=gnu99)

add_executable(bot-rwx-viewer
    main.c
    renderer_rwx.c)

pods_use_pkg_config_packages(bot-rwx-viewer bot2-vis)

pods_install_executables(bot-rwx-viewer)
