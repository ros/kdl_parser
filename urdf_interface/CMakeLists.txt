project(urdf_interface)

find_package(catkin REQUIRED COMPONENTS urdfdom)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS urdfdom
)

install(DIRECTORY include/
        DESTINATION include)
