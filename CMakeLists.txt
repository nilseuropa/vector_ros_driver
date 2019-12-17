cmake_minimum_required(VERSION 2.8.3)
project(vector_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs
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
   actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vector_ros_driver
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
