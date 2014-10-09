find_package(PythonInterp REQUIRED)

# install all python packages in the src/ directory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/src)

# executable scripts:  script-name  python-module
pods_install_python_script(bot-log2mat bot_log2mat.log_to_mat)
