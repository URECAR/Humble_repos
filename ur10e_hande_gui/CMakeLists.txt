cmake_minimum_required(VERSION 3.8)
project(ur10e_hande_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_msgs REQUIRED)

# Add sources and headers
set(SOURCES src/ur10e_hande_gui_node.cpp)
set(HEADERS include/ur10e_hande_gui/main_window.h)

add_executable(ur10e_hande_gui_node ${SOURCES} ${HEADERS})

# Set properties for the executable, including AutoMoc include paths
set_target_properties(ur10e_hande_gui_node PROPERTIES
  AUTOMOC ON
  CXX_STANDARD 17
  AUTOMOC_INCLUDE_DIRECTORIES "${rclcpp_INCLUDE_DIRS};${moveit_ros_planning_interface_INCLUDE_DIRS};${moveit_servo_INCLUDE_DIRS};${geometry_msgs_INCLUDE_DIRS};${tf2_ros_INCLUDE_DIRS};${moveit_msgs_INCLUDE_DIRS}"
)

# Add project's own include directory
target_include_directories(ur10e_hande_gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS dependencies
ament_target_dependencies(ur10e_hande_gui_node
  rclcpp
  moveit_ros_planning_interface
  moveit_servo
  geometry_msgs
  tf2_ros
  moveit_msgs
)

# Link Qt libraries
target_link_libraries(ur10e_hande_gui_node
  Qt5::Widgets
)

# Install rules
install(TARGETS
  ur10e_hande_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()