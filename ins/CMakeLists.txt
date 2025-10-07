cmake_minimum_required(VERSION 3.0.2)
project(ins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  robot_localization
)

catkin_package(
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
