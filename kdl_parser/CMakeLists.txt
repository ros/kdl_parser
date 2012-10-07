cmake_minimum_required(VERSION 2.8.3)

project(kdl_parser)

find_package(catkin REQUIRED COMPONENTS collada_parser roscpp rosconsole urdf)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS collada_parser roscpp rosconsole urdf)

include_directories(include)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_library(tinyxml_library tinyxml)

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_library} ${Orocos-KDL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(check_kdl_parser src/check_kdl_parser.cpp )
target_link_libraries(check_kdl_parser ${PROJECT_NAME})

catkin_add_gtest(test_kdl_parser test/test_kdl_parser.cpp )
target_link_libraries(test_kdl_parser ${PROJECT_NAME})

# How does CATKIN do this?
#rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)
install(TARGETS ${PROJECT_NAME} 
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
