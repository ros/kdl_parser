cmake_minimum_required(VERSION 2.8.3)
project(urdf_parser_plugin)

find_package(catkin REQUIRED urdfdom_headers)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS urdfdom_headers 
)

include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
