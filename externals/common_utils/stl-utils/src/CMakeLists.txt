file(GLOB hpp_files *.hpp)
file(GLOB hxx_files *.hxx)

# make the header public
pods_install_headers(${hpp_files} ${hxx_files} DESTINATION stl_utils)


