cmake_minimum_required(VERSION 2.8.3)
project(urdf)

find_package(Boost REQUIRED thread)
find_package(urdfdom REQUIRED)
find_package(catkin REQUIRED COMPONENTS urdfdom_headers urdf_parser_plugin pluginlib rosconsole_bridge roscpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosconsole_bridge roscpp
  DEPENDS urdfdom_headers urdfdom Boost
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(include)

include_directories(${catkin_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_library(TINYXML tinyxml)

add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${TINYXML} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES})

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

catkin_add_gtest(test_parser EXCLUDE_FROM_ALL test/test_robot_model_parser.cpp)
target_link_libraries(test_parser ${PROJECT_NAME})

# no idea how CATKIN does this
# rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_robot_model_parser.launch)

install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
