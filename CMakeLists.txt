cmake_minimum_required(VERSION 3.10.0)
project(lrm_localization_stack VERSION 0.0.1)

find_package(catkin REQUIRED COMPONENTS 
    rospy 
    roscpp
    tf
    tf2_ros
    pcl_conversions
    pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS ${REQUIRED_PACKAGES})

include_directories(${catkin_INCLUDE_DIRS} include)

# IMU Broadcaster Node
add_executable(imu_broadcaster_node 
    src/imu_broadcaster_node.cpp 
)
target_link_libraries(imu_broadcaster_node
   ${catkin_LIBRARIES}
)

# EKF Node
add_executable(ekf_node 
    src/ekf_node.cpp 
    src/localization_stack/ekf.cpp 
    src/localization_stack/conversions.cpp
)
target_link_libraries(ekf_node
   ${catkin_LIBRARIES}
   gtsam
)


# Mapping Node
add_executable(mapping_node 
    src/mapping_node.cpp 
    src/localization_stack/conversions.cpp
)
target_link_libraries(mapping_node
   ${catkin_LIBRARIES}
   gtsam
)