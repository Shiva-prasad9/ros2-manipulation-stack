cmake_minimum_required(VERSION 3.8)
project(fr3_manipulation)

# C++17 required for MoveIt Task Constructor
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# ── Pick and place executable ─────────────────────────────────────────
add_executable(pick_place_node src/pick_place_node.cpp)

ament_target_dependencies(pick_place_node
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  tf2_eigen
)

# ── Install ───────────────────────────────────────────────────────────
install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()