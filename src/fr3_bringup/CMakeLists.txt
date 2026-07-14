cmake_minimum_required(VERSION 3.8)
project(fr3_bringup)

find_package(ament_cmake REQUIRED)

# Install launch directory so ros2 launch can find it
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
