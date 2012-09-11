project(srdf)

catkin_project(${PROJECT_NAME} 
  INCLUDE_DIRS include
)

install(DIRECTORY include/
        DESTINATION include)
