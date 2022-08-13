cmake_minimum_required(VERSION 3.8)
project(launch_files)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(octomap_server REQUIRED)
find_package(engine_controller REQUIRED)
find_package(rplidar_ros2 REQUIRED)
find_package(scan_to_pointcloud2 REQUIRED)
find_package(top_to_lidar_tf_static_publisher REQUIRED)
find_package(scan_assembler REQUIRED)
find_package(bluetooth_talker REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
