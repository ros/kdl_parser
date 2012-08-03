project(kdl_parser)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include)

include_directories(include)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_library(tinyxml_library tinyxml)

find_package(ROS REQUIRED COMPONENTS roscpp rosconsole)
include_directories(${ROS_INCLUDE_DIRS})

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

find_package(urdf)
include_directories(${urdf_INCLUDE_DIRS})

find_package(collada_parser)
include_directories(${collada_parser_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_library} ${Orocos-KDL_LIBRARIES} ${collada_parser_LIBRARIES} ${urdf_LIBRARIES} ${ROS_LIBRARIES})

add_executable(check_kdl_parser src/check_kdl_parser.cpp )
target_link_libraries(check_kdl_parser ${PROJECT_NAME})

add_gtest(test_kdl_parser test/test_kdl_parser.cpp )
target_link_libraries(test_kdl_parser ${PROJECT_NAME})

# How does CATKIN do this?
#rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)
