cmake_minimum_required(VERSION 3.0.2)
project(lcm_tools)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lcm_tools
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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

add_executable(lcmnjustfull src/lcm_njust_receiverFull.cpp)
target_link_libraries(lcmnjustfull ${catkin_LIBRARIES} ${LCM_NAMESPACE}lcm)

add_executable(wheel_leg_odometry kinematic/wheel_leg_odometry.cpp)
set_property(TARGET wheel_leg_odometry PROPERTY CXX_STANDARD 11)
target_link_libraries(wheel_leg_odometry ${catkin_LIBRARIES} tinyxml)
install(TARGETS wheel_leg_odometry RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS kinematic/test/print_wheel_states.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
