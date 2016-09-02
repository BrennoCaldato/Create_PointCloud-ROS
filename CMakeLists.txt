cmake_minimum_required(VERSION 2.8.3)
project(create_pointcloud)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs laser_geometry sensor_msgs tf rospy)
find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(criar_pointcloud src/criar_pointcloud.cpp)

target_link_libraries(criar_pointcloud ${catkin_LIBRARIES})

add_executable(pointcloud_tf_broadcaster src/pointcloud_tf_broadcaster.cpp)

target_link_libraries(pointcloud_tf_broadcaster ${catkin_LIBRARIES})

add_executable(cloud_to_image src/cloud_to_image.cpp)

target_link_libraries(cloud_to_image ${PCL_LIBRARIES} ${catkin_LIBRARIES})

