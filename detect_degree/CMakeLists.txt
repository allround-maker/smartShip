cmake_minimum_required(VERSION 3.0.2)
project(detect_degree)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  std_msgs
  image_transport
)

find_package (OpenCV 4.2.0 REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES opencv
  #CATKIN_DEPENDS roscpp cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
message(STATUS ${OpenCV_INCLUDE_DIRS})

add_executable(detect_cir src/detect_cir.cpp)
target_link_libraries(detect_cir ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detect_rec src/detect_rec.cpp)
target_link_libraries(detect_rec ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detect_tri src/detect_tri.cpp)
target_link_libraries(detect_tri ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

