cmake_minimum_required(VERSION 3.5)
project(scara_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
