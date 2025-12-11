list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# for ubuntu 18.04, update gcc/g++ to 9, and download tbb2018 from
# https://github.com/oneapi-src/oneTBB/releases/download/2018/tbb2018_20170726oss_lin.tgz,
# extract it into CUSTOM_TBB_DIR 
# specifiy tbb2018, e.g. CUSTOM_TBB_DIR=/home/idriver/Documents/tbb2018_20170726oss
if (CUSTOM_TBB_DIR)
    set(TBB2018_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2018_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.7")
    include_directories(${TBB2018_INCLUDE_DIR})
    link_directories(${TBB2018_LIBRARY_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        pcl_ros
        tf
        tf2_ros
        tf2_geometry_msgs
        message_generation
        eigen_conversions
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_path(LIVOX_CUSTOM_MSG_INCLUDE_DIR
        livox_ros_driver/CustomMsg.h
        HINTS ${catkin_INCLUDE_DIRS}
)

if (LIVOX_CUSTOM_MSG_INCLUDE_DIR)
    add_definitions(-DFASTER_LIO_HAS_LIVOX)
    include_directories(${LIVOX_CUSTOM_MSG_INCLUDE_DIR})
else ()
    message(STATUS "livox_ros_driver/CustomMsg.h not found. Livox-specific features will be disabled.")
endif ()

if (EIGEN3_INCLUDE_DIR AND NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()
if (TARGET Eigen3::Eigen AND NOT EIGEN3_LIBRARIES)
    set(EIGEN3_LIBRARIES Eigen3::Eigen)
endif ()

add_message_files(
        FILES
        Pose6D.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)
catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp rospy std_msgs message_runtime pcl_ros tf tf2_ros tf2_geometry_msgs eigen_conversions
        DEPENDS EIGEN3 PCL
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
)
