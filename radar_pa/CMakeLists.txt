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

#############
## Install ##
#############

install(PROGRAMS
  scripts/extract_message.py
  scripts/radar2pcd_pa_node.py
  scripts/radar_pa_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

