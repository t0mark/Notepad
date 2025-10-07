cmake_minimum_required(VERSION 3.0.2)
project(dwa)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base
  dwa_local_planner
  pcl_ros
  nodelet
  pointcloud_to_laserscan
  costmap_2d
  dynamic_reconfigure
  geographic_msgs
  geodesy
  gazebo_ros
  gazebo_ros_control
  hardware_interface
  urdf
  xacro
)

## Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    move_base
    dwa_local_planner
    pcl_ros
    nodelet
    pointcloud_to_laserscan
    costmap_2d
    dynamic_reconfigure
    geographic_msgs
    geodesy
    gazebo_ros
    gazebo_ros_control
    hardware_interface
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Python scripts installation (only existing scripts)
catkin_install_python(
  PROGRAMS
    scripts/navigation_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install directories
install(
  DIRECTORY
    launch
    config
    urdf
    meshes
    worlds
    models
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)