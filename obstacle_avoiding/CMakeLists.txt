cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoiding)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_avoiding
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp std_masgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${PROJECT_NAME}_generate_messages_cpp) 

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner ${PROJECT_NAME}_generate_messages_cpp) 


add_executable(planner2 src/planner2.cpp)
target_link_libraries(planner2 ${catkin_LIBRARIES})
add_dependencies(planner2 ${PROJECT_NAME}_generate_messages_cpp) 


add_executable(planner2_hopping src/planner2_hopping.cpp)
target_link_libraries(planner2_hopping ${catkin_LIBRARIES})
add_dependencies(planner2_hopping ${PROJECT_NAME}_generate_messages_cpp) 