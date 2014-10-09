# require python
find_package(PythonInterp REQUIRED)
# write python file in python/
file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/path_util/path_util.py "def getBasePath(): return '${CMAKE_INSTALL_PREFIX}'\n"
                               "def getDataPath(): return '${CMAKE_INSTALL_PREFIX}/data'\n"
                               "def getConfigPath(): return '${CMAKE_INSTALL_PREFIX}/config'\n"
                               "def getModelsPath(): return '${CMAKE_INSTALL_PREFIX}/models'\n")

# install the python file
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR})

# create a generic python launcher script launcher that sets the PYTHONPATH to the pods
# python module install path
set(script_name "python-pods")
# which python version?
execute_process(COMMAND 
    ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
    OUTPUT_VARIABLE pyversion)

# where do we install .py files to?
set(python_install_dir 
    ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/dist-packages)
set(python_old_install_dir 
    ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/site-packages)

# write the script file
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${script_name} "#!/bin/sh\n"
    "export PYTHONPATH=${python_install_dir}:${python_old_install_dir}:\${PYTHONPATH}\n"
    "exec ${PYTHON_EXECUTABLE} $*\n")

# install it...
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${script_name} DESTINATION bin)