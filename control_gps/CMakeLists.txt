cmake_minimum_required(VERSION 3.0.2)
project(control_gps)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp geometry_msgs sensor_msgs)

catkin_package(
  #LIBRARIES control_pkg
  #CATKIN_DEPENDS std_msgs roscpp geometry_msgs sensor_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(gps_controller src/gps_controller.cpp)
add_dependencies(gps_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_controller ${catkin_LIBRARIES})
