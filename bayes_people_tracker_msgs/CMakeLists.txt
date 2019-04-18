cmake_minimum_required(VERSION 2.8.3)
project(bayes_people_tracker_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    people_msgs
    std_msgs
    visualization_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  PeopleTracker.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs visualization_msgs
)

