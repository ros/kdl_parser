cmake_minimum_required(VERSION 2.8.3)
project(joint_state_publisher)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS joint_state_publisher/joint_state_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
