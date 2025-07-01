cmake_minimum_required(VERSION 3.0.2)
project(lcm_tools)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(lcm REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lcm_tools
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


include(${LCM_USE_FILE})
include_directories(src/exlcm)
include_directories(include/lcm_tools)

add_executable(lcmpublisher src/lcm_publisher.cpp)
target_link_libraries(lcmpublisher ${catkin_LIBRARIES} ${LCM_NAMESPACE}lcm)

add_executable(lcmreceiver src/lcm_receiver.cpp)
target_link_libraries(lcmreceiver ${catkin_LIBRARIES} ${LCM_NAMESPACE}lcm)

add_executable(lcmnjust src/lcm_njust_receiver.cpp)
target_link_libraries(lcmnjust ${catkin_LIBRARIES} ${LCM_NAMESPACE}lcm)
