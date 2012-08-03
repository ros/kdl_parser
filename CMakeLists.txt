cmake_minimum_required(VERSION 2.8)
project(robot_model)

find_package(catkin REQUIRED)

catkin_stack()

add_subdirectory(resource_retriever)
add_subdirectory(srdf)
add_subdirectory(urdf_interface)
add_subdirectory(urdf_parser)
add_subdirectory(collada_parser)
add_subdirectory(urdf)
add_subdirectory(collada_urdf)
add_subdirectory(kdl_parser)

#add_subdirectory(convex_decomposition)
#add_subdirectory(ivcon)
#add_subdirectory(simmechanics_to_urdf)
