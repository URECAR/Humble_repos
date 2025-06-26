cmake_minimum_required(VERSION 3.22)
project(ur10e_hande_mtc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(cartesian src/cartesian.cpp)

# Target dependencies
ament_target_dependencies(cartesian
  rclcpp
  moveit_core
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2_geometry_msgs
  tf2_ros
)

# Install executable
install(TARGETS
  cartesian
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files (include *.yaml and *.rviz)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.rviz"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
