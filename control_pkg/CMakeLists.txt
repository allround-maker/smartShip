cmake_minimum_required(VERSION 3.0.2)
project(control_pkg)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp geometry_msgs)

add_message_files(FILES order.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES control_pkg
  CATKIN_DEPENDS std_msgs roscpp geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_publisher src/test_publisher.cpp)
add_dependencies(test_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_publisher ${catkin_LIBRARIES})

add_executable(motor_controller src/motor_controller.cpp)
add_dependencies(motor_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller ${catkin_LIBRARIES})

add_executable(test_order_pub src/test_order_pub.cpp)
add_dependencies(test_order_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_order_pub ${catkin_LIBRARIES})


add_executable(drive_algo src/drive_algo.cpp)
add_dependencies(drive_algo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_algo ${catkin_LIBRARIES})

#add_executable(drive_algo2 src/drive_algo2.cpp)
#add_dependencies(drive_algo2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(drive_algo2 ${catkin_LIBRARIES})

add_executable(drive_algo3 src/drive_algo3.cpp)
add_dependencies(drive_algo3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_algo3 ${catkin_LIBRARIES})

add_executable(stepper src/stepper.cpp)
add_dependencies(stepper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stepper ${catkin_LIBRARIES})

add_executable(stepper1 src/stepper1.cpp)
add_dependencies(stepper1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stepper1 ${catkin_LIBRARIES})

add_executable(stepper2 src/stepper2.cpp)
add_dependencies(stepper2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stepper2 ${catkin_LIBRARIES})

add_executable(stepper3 src/stepper3.cpp)
add_dependencies(stepper3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stepper3 ${catkin_LIBRARIES})