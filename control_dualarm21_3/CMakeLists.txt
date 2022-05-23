cmake_minimum_required(VERSION 3.1.3)
project(control_dualarm21_3)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
     tf2_ros
    position_msgs
)
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
   
  DEPENDS
)



include_directories(
	${catkin_INCLUDE_DIRS}
)
add_executable(move_group_node test/move_group_interface_demo.cpp)
target_link_libraries(move_group_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(motion_planning_node test/motion_planning_api.cpp)
target_link_libraries(motion_planning_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(my_control_test test/my_control_test.cpp)
target_link_libraries(my_control_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS my_control_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dual_arm_control test/dual_arm_control.cpp)
target_link_libraries(dual_arm_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS dual_arm_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(ai_connect test/ai_connect.cpp)
target_link_libraries(ai_connect ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ai_connect DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
