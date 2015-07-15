cmake_minimum_required(VERSION 2.8.3)
project(collada_urdf)

find_package(catkin REQUIRED COMPONENTS angles collada_parser resource_retriever urdf geometric_shapes tf cmake_modules)

find_package(TinyXML REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS angles collada_parser resource_retriever urdf geometric_shapes tf)

include_directories(include)

find_package(assimp QUIET)
if ( NOT ASSIMP_FOUND )
  find_package(Assimp QUIET)
  if ( NOT ASSIMP_FOUND )
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ASSIMP assimp)
  endif()
endif()
if( ASSIMP_FOUND )
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150" )
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
  endif()
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.885" )
    add_definitions(-DASSIMP_EXPORT_API)
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()

# Note: assimp 3.1 overwrites CMake Boost variables, so we need to check for
# Boost after assimp.
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
include_directories(${Boost_INCLUDE_DIR})

find_package(COLLADA_DOM 2.3 COMPONENTS 1.5)
if( COLLADA_DOM_FOUND )
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  link_directories(${COLLADA_DOM_LIBRARY_DIRS})
endif()

include_directories(${TinyXML_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/collada_urdf.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${COLLADA_DOM_LIBRARIES}  
  ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILER_FLAGS "${ASSIMP_CXX_FLAGS} ${ASSIMP_CFLAGS_OTHER}")
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${ASSIMP_LINK_FLAGS}") 

add_executable(urdf_to_collada src/urdf_to_collada.cpp)
target_link_libraries(urdf_to_collada ${catkin_LIBRARIES} ${COLLADA_DOM_LIBRARIES}  
  ${Boost_LIBRARIES} ${PROJECT_NAME})

add_executable(collada_to_urdf src/collada_to_urdf.cpp)
target_link_libraries(collada_to_urdf ${ASSIMP_LIBRARIES} ${catkin_LIBRARIES} ${COLLADA_DOM_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(collada_to_urdf PROPERTIES COMPILER_FLAGS "${ASSIMP_CXX_FLAGS} ${ASSIMP_CFLAGS_OTHER}")
set_target_properties(collada_to_urdf PROPERTIES LINK_FLAGS "${ASSIMP_LINK_FLAGS}")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_collada_writer test/test_collada_urdf.cpp)
  target_link_libraries(test_collada_writer ${PROJECT_NAME} ${catkin_LIBRARIES} ${COLLADA_DOM_LIBRARIES}  
    ${Boost_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} urdf_to_collada collada_to_urdf
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
