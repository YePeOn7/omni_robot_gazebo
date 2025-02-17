cmake_minimum_required(VERSION 3.0.2)
project(omni_robot_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(set_goal src/set_goal.cpp)
target_link_libraries(set_goal ${catkin_LIBRARIES})

add_executable(set_goal2 src/set_goal2.cpp)
target_link_libraries(set_goal2 ${catkin_LIBRARIES})