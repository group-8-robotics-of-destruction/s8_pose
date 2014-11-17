set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_pose)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp s8_common_node s8_utils s8_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp geometry_msgs s8_common_node s8_utils s8_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_pose_node src/pose_node.cpp)
target_link_libraries(s8_pose_node ${catkin_LIBRARIES})
