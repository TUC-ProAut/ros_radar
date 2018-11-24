cmake_minimum_required(VERSION 2.8.3)
project(radar_pa)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  can_msgs
  radar_pa_msgs
)

catkin_package(

  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    can_msgs
    radar_pa_msgs
)

#############
##  Build  ##
#############

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############

