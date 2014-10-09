find_package(PythonInterp REQUIRED)

option(GEN_BUILD_PREFIX "Generate build_prefix.py" ON)

if(GEN_BUILD_PREFIX)
  file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/src/bot_procman/build_prefix.py
      "BUILD_PREFIX='${CMAKE_INSTALL_PREFIX}'\n")
    #  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/bot_procman/build_prefix.py)
endif(GEN_BUILD_PREFIX)

# install all python packages in the src/ directory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/src)

# executable scripts:  script-name  python-module
pods_install_python_script(bot-procman-sheriff bot_procman.sheriff_gtk.sheriff_gtk)

install(FILES procman-sheriff.glade DESTINATION share/bot_procman)
