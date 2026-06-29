cmake_minimum_required(VERSION 3.8)
project(fr3_moveit_config)

# Find the ament build system
find_package(ament_cmake REQUIRED)

# Install config/ and launch/ into the package share directory
# so move_group.launch.py can locate the YAML files at runtime
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()