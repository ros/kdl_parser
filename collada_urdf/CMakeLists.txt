cmake_minimum_required(VERSION 2.8.3)
project(collada_urdf)

find_package(catkin REQUIRED COMPONENTS angles collada_parser resource_retriever urdf)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS angles collada_parser resource_retriever urdf)

include_directories(include)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})

find_package(assimp QUIET)
if ( NOT assimp_FOUND )
  find_package(Assimp QUIET)
  if ( Assimp_FOUND )
    set(assimp_FOUND ${Assimp_FOUND})
    set(assimp_LIBRARIES ${Assimp_LIBRARIES})
    set(assimp_LIBRARY_DIRS ${Assimp_LIBRARY_DIRS})
    set(assimp_INCLUDE_DIRS ${Assimp_INCLUDE_DIRS})
    set(assimp_VERSION ${Assimp_VERSION})
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(assimp assimp)
  endif()
endif()
if( assimp_FOUND )
  if( ${assimp_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
  endif()
  include_directories(${assimp_INCLUDE_DIRS})
  link_directories(${assimp_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(assimp_LIBRARIES assimp)
  set(assimp_LIBRARY_DIRS)
  set(assimp_CXX_FLAGS)
  set(assimp_CFLAGS_OTHER)
  set(assimp_LINK_FLAGS)
  set(assimp_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

find_library(tinyxml_library tinyxml)

find_package(COLLADA_DOM 2.3 COMPONENTS 1.5)
if( COLLADA_DOM_FOUND )
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  link_directories(${COLLADA_DOM_LIBRARY_DIRS})
endif()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/collada_urdf.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_library} ${assimp_LIBRARIES} ${catkin_LIBRARIES} ${COLLADA_DOM_LIBRARIES}  
  ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILER_FLAGS "${assimp_CXX_FLAGS} ${assimp_CFLAGS_OTHER}")
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${assimp_LINK_FLAGS}") 

add_executable(urdf_to_collada src/urdf_to_collada.cpp)
target_link_libraries(urdf_to_collada ${PROJECT_NAME})

catkin_add_gtest(test_collada_writer test/test_collada_urdf.cpp)
target_link_libraries(test_collada_writer ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} urdf_to_collada
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
