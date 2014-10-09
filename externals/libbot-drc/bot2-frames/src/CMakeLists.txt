add_definitions(-std=gnu99)
# Create a shared library libbot2-frames.so with a single source file
add_library(bot2-frames SHARED  bot_frames.c)
set(REQUIRED_LIBS bot2-core bot2-param-client lcmtypes_bot2-frames)

pods_use_pkg_config_packages(bot2-frames ${REQUIRED_LIBS})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(bot2-frames PROPERTIES SOVERSION 2)

# make the header public
# install it to include/bot2-frames
pods_install_headers(bot_frames.h DESTINATION bot_frames)

# make the library public
pods_install_libraries(bot2-frames)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(bot2-frames
    CFLAGS
    LIBS -lbot2-frames
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
