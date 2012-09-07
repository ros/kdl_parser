project(resource_retriever)

catkin_project(${PROJECT_NAME} 
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

add_subdirectory(test EXCLUDE_FROM_ALL)

install(TARGETS ${PROJECT_NAME} 
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
