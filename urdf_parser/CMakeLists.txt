cmake_minimum_required(VERSION 2.8.3)
project(urdf_parser)

find_package(catkin REQUIRED COMPONENTS urdfdom)

catkin_package(
  DEPENDS urdfdom
  )
