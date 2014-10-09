cmake_minimum_required(VERSION 2.6.0)

include(cmake/pods.cmake)

add_subdirectory(src/logfilter)
add_subdirectory(src/logsplice)
add_subdirectory(src/who)
add_subdirectory(src/tunnel)
add_subdirectory(python)


# bot-spy
set(script_name bot-spy)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${script_name} 
    "#!/bin/sh\n"
    "CLASSPATH=`PKG_CONFIG_PATH=PKG_CONFIG_PATH:${CMAKE_INSTALL_PREFIX}/lib/pkgconfig pkg-config --variable=classpath lcm-java`\n"
    "for d in . .. ../.. ../../.. ../../../.. \"${CMAKE_INSTALL_PREFIX}\"; do\n"
    #    "for d in \"${CMAKE_INSTALL_PREFIX}\"; do\n"
    "  if [ -d $d/share/java ]; then\n"
    "    jd=$d/share/java\n"
    "    echo Checking $jd\n"
    "    for f in $jd/lcmtypes_*.jar $jd/lcmspy_plugins_*.jar; do\n"
    "      if [ -e $f ]; then\n"
    "        echo \"   Found $f\"\n"
    "        CLASSPATH=\$CLASSPATH:\$f\n"
    "      fi\n"
    "    done\n"
    "  fi\n"
    "done\n"
    "exec java -ea -cp \$CLASSPATH lcm.spy.Spy $*\n")

# install it...
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${script_name} DESTINATION bin)
