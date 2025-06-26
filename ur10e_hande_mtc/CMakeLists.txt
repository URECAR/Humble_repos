cmake_minimum_required(VERSION 3.22)
project(ur10e_hande_mtc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

# Add executable
add_executable(cartesian src/cartesian.cpp)

# Target dependencies
ament_target_dependencies(cartesian
  rclcpp
  moveit_task_constructor_core
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

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.rviz"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
