cmake_minimum_required(VERSION 2.8.3)
project(srdf)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY include/
        DESTINATION include)
