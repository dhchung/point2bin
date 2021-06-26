cmake_minimum_required(VERSION 3.0)
project(point2bin)

set(SOURCE
    "src/main.cpp")

file(GLOB CLASS_SOURCES
     "classes/*.cpp"
     "include/*.h")

     
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             std_msgs
             rosbag
             geometry_msgs
             sensor_msgs
             pcl_ros
             pcl_conversions
             pcl_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        std_msgs
        rosbag
        geometry_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        pcl_msgs
)
         
         
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)


add_executable(save_ptcloud
               ${SOURCE}
               ${CLASS_SOURCES})

target_link_libraries (save_ptcloud
                       ${catkin_LIBRARIES})