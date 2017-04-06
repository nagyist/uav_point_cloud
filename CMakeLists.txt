cmake_minimum_required(VERSION 2.8.3)
project(uav_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  laser_geometry
)
 
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp tf sensor_msgs laser_geometry
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/Node.h
)

set(CPP_FILES
    src/Node.cpp
)

add_executable(conversor_node src/conversor_node.cpp ${CPP_FILES})
add_dependencies(conversor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(conversor_node ${catkin_LIBRARIES})
