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
  gazebo_plugins
  urdf
  xacro
)

## Find Gazebo
find_package(gazebo REQUIRED)

## Catkin package configuration
catkin_package(
  INCLUDE_DIRS plugins/include
  LIBRARIES gazebo_ros_ouster_gpu_laser gazebo_ros_ouster_laser
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
    gazebo_plugins
  DEPENDS
    gazebo
)

## Include directories
include_directories(
  plugins/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Build Ouster Gazebo Plugin (GPU version - Gepetto 방식)
add_library(gazebo_ros_ouster_gpu_laser
  plugins/src/GazeboRosOusterLaser.cpp
)

target_compile_definitions(gazebo_ros_ouster_gpu_laser
  PRIVATE GAZEBO_GPU_RAY=1
)

target_link_libraries(gazebo_ros_ouster_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)

add_dependencies(gazebo_ros_ouster_gpu_laser
  ${catkin_EXPORTED_TARGETS}
)

## Build Ouster Gazebo Plugin (CPU version)
add_library(gazebo_ros_ouster_laser
  plugins/src/GazeboRosOusterLaser.cpp
)

target_compile_definitions(gazebo_ros_ouster_laser
  PRIVATE GAZEBO_GPU_RAY=0
)

target_link_libraries(gazebo_ros_ouster_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_dependencies(gazebo_ros_ouster_laser
  ${catkin_EXPORTED_TARGETS}
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

## Install Gazebo plugin libraries
install(TARGETS gazebo_ros_ouster_gpu_laser gazebo_ros_ouster_laser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install plugin headers
install(DIRECTORY plugins/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)