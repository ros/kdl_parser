cmake_minimum_required(VERSION 2.8.3)

project(kdl_parser)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED
  COMPONENTS roscpp rosconsole urdf cmake_modules
)
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${KDL_LIBRARY}
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosconsole urdf
  DEPENDS orocos_kdl TinyXML
)

add_library(${PROJECT_NAME} src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME}
  ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
)

add_executable(check_kdl_parser src/check_kdl_parser.cpp )
target_link_libraries(check_kdl_parser ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest_gtest(test_kdl_parser test/test_kdl_parser.launch test/test_kdl_parser.cpp )
  target_link_libraries(test_kdl_parser ${PROJECT_NAME})
endif()

# How does CATKIN do this?
#rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)
install(TARGETS ${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
