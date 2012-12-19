cmake_minimum_required(VERSION 2.8.3)
project(collada_parser)

find_package(Boost REQUIRED system)

find_package(catkin REQUIRED COMPONENTS urdfdom_headers roscpp)

catkin_package( 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS urdfdom_headers roscpp)

include_directories(${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake-extensions/)
find_package(PkgConfig)
find_package(COLLADA_DOM 2.3 COMPONENTS 1.5)
if(COLLADA_DOM_FOUND)
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  link_directories(${COLLADA_DOM_LIBRARY_DIRS})
endif()

# necessary for collada reader to create the temporary dae files due
# to limitations in the URDF geometry
include (CheckFunctionExists)
check_function_exists(mkstemps HAVE_MKSTEMPS)
if(HAVE_MKSTEMPS)
  add_definitions("-DHAVE_MKSTEMPS")
endif()

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/collada_parser.cpp)
target_link_libraries(${PROJECT_NAME}
  ${COLLADA_DOM_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES COMPILER_FLAGS "${COLLADA_DOM_CFLAGS_OTHER}"
)
if(APPLE) 
  set_target_properties(${PROJECT_NAME}
    PROPERTIES LINK_FLAGS
    "${COLLADA_DOM_LDFLAGS_OTHER} -undefined dynamic_lookup"
  )
else()
  set_target_properties(${PROJECT_NAME}
    PROPERTIES LINK_FLAGS "${COLLADA_DOM_LDFLAGS_OTHER}"
  )
endif()

install(TARGETS ${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
