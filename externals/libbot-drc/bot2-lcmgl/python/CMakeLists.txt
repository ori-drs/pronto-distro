find_package(PythonInterp REQUIRED)

# install all python packages in the src/ directory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/src)

# executable scripts:
#pods_install_python_script(script-name python-module)

