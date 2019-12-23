cmake_minimum_required(VERSION 2.8.3)
project(vector_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib
  actionlib_msgs
  tf
  tf2
)

catkin_python_setup()

add_service_files(
   FILES
   BatteryState.srv
   AnimList.srv
   HeadAngle.srv
   LiftHeight.srv
   SayText.srv
)

add_action_files(
   FILES
   PlayAnimation.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
