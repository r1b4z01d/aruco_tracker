cmake_minimum_required(VERSION 2.8.3)
project(aruco_tracker)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)
catkin_install_python(PROGRAMS
  aruco_tracker/aruco_tracker.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/aruco_tracker
)

