cmake_minimum_required(VERSION 2.8.3)
project(resource_retriever)

find_package(catkin REQUIRED COMPONENTS rosconsole roslib)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include)

include_directories(include)

find_package(catkin REQUIRED COMPONENTS rosconsole roslib)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include(FindCURL)
if(NOT CURL_FOUND)
  message("CURL not found!  Aborting...")
  fail()
endif(NOT CURL_FOUND)
include_directories(${CURL_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/retriever.cpp)
target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test EXCLUDE_FROM_ALL)
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
