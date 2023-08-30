cmake_minimum_required(VERSION 3.0.2)
project(robotics_hackathon_automation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
)


add_message_files(
  FILES
  point.msg
  PointArray.msg

)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)



catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
