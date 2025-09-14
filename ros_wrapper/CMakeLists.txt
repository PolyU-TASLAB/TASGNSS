cmake_minimum_required(VERSION 3.0.2)
project(ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GnssObservation.msg
  GnssEphemMsg.msg
  GnssProcessedData.msg
  GnssTimeMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)