cmake_minimum_required(VERSION 3.8)
project(slam_omokai)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
