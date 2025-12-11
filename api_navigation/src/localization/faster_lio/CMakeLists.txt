add_library(${PROJECT_NAME}
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
        )

set(_faster_lio_dependencies ${PROJECT_NAME}_gencpp)
if (TARGET livox_ros_driver_gencpp)
    list(APPEND _faster_lio_dependencies livox_ros_driver_gencpp)
else ()
    message(WARNING "livox_ros_driver_gencpp target not found; assuming the message headers are provided by a binary install.")
endif ()
add_dependencies(${PROJECT_NAME} ${_faster_lio_dependencies})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
