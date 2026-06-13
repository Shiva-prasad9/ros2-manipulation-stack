cmake_minimum_required(VERSION 3.8)
project(fr3_simulation)

# Find the ament build system — required for all ROS 2 cmake packages
find_package(ament_cmake REQUIRED)

# Install launch/ and worlds/ directories into the package share folder
# so ros2 launch and get_package_share_directory() can locate them
# at runtime after colcon build
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
