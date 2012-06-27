cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_add_boost_directories()

set(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake-extensions/ )
find_package(PkgConfig)
find_package(COLLADA_DOM 2.3 COMPONENTS 1.5)
if( COLLADA_DOM_FOUND )
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  #link_directories(${COLLADA_DOM_LIBRARY_DIRS})
endif()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# necessary for collada reader to create the temporary dae files due to limitations in the URDF geometry
check_function_exists(mkstemps HAVE_MKSTEMPS)
if( HAVE_MKSTEMPS )
  add_definitions("-DHAVE_MKSTEMPS")
endif()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/collada_parser.cpp src/hack.cpp)

rosbuild_add_compile_flags(${PROJECT_NAME} "${COLLADA_DOM_CFLAGS_OTHER}")
rosbuild_add_link_flags(${PROJECT_NAME} "${COLLADA_DOM_LDFLAGS_OTHER}")

if(APPLE) 
  rosbuild_add_link_flags(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup") 
endif(APPLE)

#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_link_boost(${PROJECT_NAME} thread)

