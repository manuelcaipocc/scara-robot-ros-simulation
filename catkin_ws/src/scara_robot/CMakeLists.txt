cmake_minimum_required(VERSION 3.0.2)
project(scara_robot)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
