cmake_minimum_required(VERSION 2.8.3)
project(posmv)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  project11_msgs
  sensor_msgs
  rospy
  diagnostic_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs project11_msgs sensor_msgs rospy diagnostic_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

