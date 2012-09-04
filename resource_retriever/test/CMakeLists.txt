set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_gtest(${PROJECT_NAME}_utest test.cpp)
target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})
