cmake_minimum_required(VERSION 3.0.2)
project(autonomous_navigation)

find_package(catkin REQUIRED COMPONENTS
  depthimage_to_laserscan
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  turtlebot_bringup
  turtlebot_teleop
)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  roscpp 
  sensor_msgs 
  tf
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

install(
  DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
