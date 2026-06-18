cmake_minimum_required(VERSION 3.8)
project(fr3_simulation)

# Find the ament build system
find_package(ament_cmake REQUIRED)

# Install launch/, worlds/, and config/ into the package share directory
# so get_package_share_directory() can locate them at runtime
install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
