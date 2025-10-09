cmake_minimum_required(VERSION 3.0.2)
project(kakao_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/simulator/gps_publisher.py
  scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz scripts web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
