cmake_minimum_required(VERSION 2.8.3)
project(radar_pa_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

 add_message_files(
   FILES
    radar_msg_A.msg
    radar_msg_B.msg
    radar_msg.msg
 )

 generate_messages(
   DEPENDENCIES
     std_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    std_msgs
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
