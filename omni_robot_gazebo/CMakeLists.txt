cmake_minimum_required(VERSION 2.8.3)
project(omni_robot_gazebo)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install
foreach(dir config/joy launch launch/joy worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
#if (CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch)
#  roslaunch_add_file_check(launch/joy)
#endif()

add_executable(omni_odometry src/omni_odometry.cpp)
target_link_libraries(omni_odometry ${catkin_LIBRARIES})

