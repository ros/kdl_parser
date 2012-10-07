cmake_minimum_required(VERSION 2.8.3)
project(urdf)

find_package(catkin REQUIRED COMPONENTS urdfdom urdfdom_headers collada_parser rosconsole_bridge roscpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS urdfdom urdfdom_headers collada_parser rosconsole_bridge roscpp)

include_directories(include)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS roscpp rosconsole_bridge rostime)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_library(TINYXML tinyxml)

add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${urdfdom_LIBRARIES} ${collada_parser_LIBRARIES} ${rosconsole_bridge_LIBRARIES} ${TINYXML} ${catkin_LIBRARIES})

if(APPLE) 
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup") 
endif(APPLE)

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf ${PROJECT_NAME})

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz ${PROJECT_NAME})

catkin_add_gtest(test_parser EXCLUDE_FROM_ALL test/test_robot_model_parser.cpp)
target_link_libraries(test_parser ${PROJECT_NAME})

# no idea how CATKIN does this
# rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_robot_model_parser.launch)

install(TARGETS ${PROJECT_NAME} check_urdf urdf_to_graphiz
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)
