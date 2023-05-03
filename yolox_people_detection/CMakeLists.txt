cmake_minimum_required(VERSION 2.8.12)
project(yolox_people_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  clf_object_recognition_yolox
)

catkin_package()

install(PROGRAMS
  scripts/yolox_person_detection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)