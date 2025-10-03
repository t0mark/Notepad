cmake_minimum_required(VERSION 3.0.2)
project(integrated_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    tf
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 스크립트 설치 (integrated_navigation 전용)
catkin_install_python(
  PROGRAMS
    scripts/make_frame.py
    scripts/initialize_pose.py
    scripts/path_visualizer.py
    scripts/move_front.py
    scripts/sync.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Launch 파일 설치
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)