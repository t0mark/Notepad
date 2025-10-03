cmake_minimum_required(VERSION 3.0.2)
project(kakao_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 스크립트 설치 (kakao_api 전용)
catkin_install_python(
  PROGRAMS
    scripts/gps_server.py
    scripts/waypoints_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Web 폴더 및 launch 파일 설치
install(
  DIRECTORY
    web
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)