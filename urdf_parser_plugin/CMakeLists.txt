cmake_minimum_required(VERSION 2.8.3)
project(urdf_parser_plugin)

find_package(catkin REQUIRED)
find_package(urdfdom_headers REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS urdfdom_headers 
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
