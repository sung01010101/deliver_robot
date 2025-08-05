cmake_minimum_required(VERSION 3.8)
project(deliver_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python src files
install(PROGRAMS
  src/demo_cmd_vel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_cmd_vel
)

install(PROGRAMS
  src/esp32_serial_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_serial_node
)

install(PROGRAMS
  src/get_amcl_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME get_amcl_pose
)

# install(PROGRAMS
#   src/nav_through_poses.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME nav_through_poses
# )

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install description files (URDF)
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

# Install RViz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install shell scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
