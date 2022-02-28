cmake_minimum_required(VERSION 3.0.2)
project(unity_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ovis_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs ovis_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(unity_bridge src/bridge.cpp)
target_link_libraries(unity_bridge ${catkin_LIBRARIES})