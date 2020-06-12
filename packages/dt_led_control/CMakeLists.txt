cmake_minimum_required(VERSION 2.8.3)
project(dt_led_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
