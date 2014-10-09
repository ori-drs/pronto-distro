add_executable(eigen-test-triangle-view test_triangle_view.cpp)
pods_use_pkg_config_packages(eigen-test-triangle-view eigen-utils)

add_executable(eigen-test-linalg test_linalg.cpp)
pods_use_pkg_config_packages(eigen-test-linalg eigen-utils)

add_executable(eigen-test-geom test_geom.cpp)
pods_use_pkg_config_packages(eigen-test-geom eigen-utils)

add_executable(eigen-test-rigidbody test_rigidbody.cpp)
pods_use_pkg_config_packages(eigen-test-rigidbody eigen-utils)

add_executable(eigen-test-select-block test_select_block.cpp)
pods_use_pkg_config_packages(eigen-test-select-block eigen-utils)

add_executable(eigen-test-lcm-marshalling test_lcm_marshalling.cpp)
pods_use_pkg_config_packages(eigen-test-lcm-marshalling eigen-utils)

add_executable(eigen-test-file-io test_file_io.cpp)
pods_use_pkg_config_packages(eigen-test-file-io eigen-utils)

add_executable(eigen-test-lcm-matlab-rpc test_lcm_matlab_rpc.cpp)
pods_use_pkg_config_packages(eigen-test-lcm-matlab-rpc eigen-utils)

