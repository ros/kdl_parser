cmake_minimum_required(VERSION 2.8.3)
project(urdf)

find_package(Boost REQUIRED thread)
find_package(urdfdom REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  urdfdom_headers urdf_parser_plugin pluginlib rosconsole_bridge roscpp cmake_modules)

find_package(TinyXML REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ${TinyXML_INCLUDE_DIRS}
  CATKIN_DEPENDS rosconsole_bridge roscpp
  DEPENDS urdfdom_headers urdfdom Boost
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES})

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest_gtest(test_urdf_parser test/test_robot_model_parser.launch test/test_robot_model_parser.cpp)
  target_link_libraries(test_urdf_parser ${PROJECT_NAME})
endif()

# no idea how CATKIN does this
# rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_robot_model_parser.launch)

install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
