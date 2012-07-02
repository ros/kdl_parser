cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()

rosbuild_gensrv()

find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS})

find_package(rosconsole_bridge REQUIRED)
include_directories(${rosconsole_bridge_INCLUDE_DIRS})

rosbuild_add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${urdfdom_LIBRARIES} ${rosconsole_bridge_LIBRARIES})

if(APPLE) 
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup") 
endif(APPLE)

rosbuild_add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf ${PROJECT_NAME})

rosbuild_add_executable(test_parser EXCLUDE_FROM_ALL test/test_robot_model_parser.cpp)
target_link_libraries(test_parser ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_parser)
rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_robot_model_parser.launch)

