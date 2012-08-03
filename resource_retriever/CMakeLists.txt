project(resource_retriever)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include)

include_directories(include)

find_package(ROS REQUIRED COMPONENTS rosconsole roslib)
include_directories(${ROS_INCLUDE_DIRS})

include(FindCURL)
if(NOT CURL_FOUND)
  message("CURL not found!  Aborting...")
  fail()
endif(NOT CURL_FOUND)
include_directories(${CURL_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/retriever.cpp)
target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES} ${ROS_LIBRARIES})

add_subdirectory(test EXCLUDE_FROM_ALL)

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
