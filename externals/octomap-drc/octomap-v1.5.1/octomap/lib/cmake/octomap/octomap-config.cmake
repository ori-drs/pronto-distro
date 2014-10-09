# - Config file for the OctoMap package
# (example from http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file)
# It defines the following variables
#  OCTOMAP_INCLUDE_DIRS - include directories for OctoMap
#  OCTOMAP_LIBRARY_DIRS - library directories for OctoMap (normally not used!)
#  OCTOMAP_LIBRARIES    - libraries to link against
 
# Tell the user project where to find our headers and libraries
set(OCTOMAP_INCLUDE_DIRS /home/mfallon/pronto/externals/octomap-drc/octomap-v1.5.1/octomap/include)
set(OCTOMAP_LIBRARY_DIRS /home/mfallon/pronto/externals/octomap-drc/octomap-v1.5.1/octomap/lib)
 
# Our library dependencies (contains definitions for IMPORTED targets)
# include("/FooBarLibraryDepends.cmake")
 
set(OCTOMAP_LIBRARIES octomap octomath)

#set(FOOBAR_EXECUTABLE bar)
