cmake_minimum_required(VERSION 2.8.3)
project(bayes_people_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    bayes_tracking
    geometry_msgs
    message_generation
    people_msgs
    roscpp
    std_msgs
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    bayes_people_tracker_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bayes_people_tracker_msgs bayes_tracking geometry_msgs roscpp std_msgs tf visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(bayes_people_tracker
  src/bayes_people_tracker/people_tracker.cpp
)

add_dependencies(bayes_people_tracker ${catkin_EXPORTED_TARGETS})

target_link_libraries(bayes_people_tracker
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS bayes_people_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/test_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
