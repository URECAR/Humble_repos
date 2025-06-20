cmake_minimum_required(VERSION 3.5)
project(ur10e_hande_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
