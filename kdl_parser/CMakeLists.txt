cmake_minimum_required(VERSION 3.5)

project(kdl_parser)

find_package(ament_cmake_ros REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(rcutils REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_library(
  ${PROJECT_NAME}
  src/kdl_parser.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  orocos-kdl
  urdfdom_headers::urdfdom_headers)
target_link_libraries(${PROJECT_NAME} PRIVATE
  rcutils::rcutils
  urdf::urdf)

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "KDL_PARSER_BUILDING_DLL")
endif()

install(
  TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_inertia_rpy test/test_inertia_rpy.cpp)
  target_link_libraries(test_inertia_rpy ${PROJECT_NAME})

  ament_add_gtest(test_kdl_parser test/test_kdl_parser.cpp)
  target_link_libraries(test_kdl_parser ${PROJECT_NAME})
endif()

ament_export_dependencies(orocos_kdl_vendor)
ament_export_dependencies(orocos_kdl)
ament_export_dependencies(rcutils)
ament_export_dependencies(urdfdom_headers)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
