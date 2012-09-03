project(collada_urdf)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS collada_parser urdf)

include_directories(include)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})

find_package(assimp QUIET)
if( NOT assimp_FOUND )
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ASSIMP assimp)
endif()
if( ASSIMP_FOUND )
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

find_library(tinyxml_library tinyxml)

find_package(COLLADA_DOM 2.3 COMPONENTS 1.5)
if( COLLADA_DOM_FOUND )
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  link_directories(${COLLADA_DOM_LIBRARY_DIRS})
endif()

find_package(urdf)
include_directories(${urdf_INCLUDE_DIRS})
link_directories(${urdf_LIBRARY_DIRS})

find_package(resource_retriever)
include_directories(${resource_retriever_INCLUDE_DIRS})
link_directories(${resource_retriever_LIBRARY_DIRS})

find_package(geometry_angles_utils)
include_directories(${geometry_angles_utils_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/collada_urdf.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_library} ${ASSIMP_LIBRARIES} ${collada_parser_LIBRARIES} ${COLLADA_DOM_LIBRARIES} ${urdf_LIBRARIES} 
  ${resource_retriever_LIBRARIES} ${geometry_angles_utils_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILER_FLAGS "${ASSIMP_CXX_FLAGS} ${ASSIMP_CFLAGS_OTHER}")
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${ASSIMP_LINK_FLAGS}") 

add_executable(urdf_to_collada src/urdf_to_collada.cpp)
target_link_libraries(urdf_to_collada ${PROJECT_NAME})

add_gtest(test_collada_writer test/test_collada_urdf.cpp)
target_link_libraries(test_collada_writer ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} urdf_to_collada
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
