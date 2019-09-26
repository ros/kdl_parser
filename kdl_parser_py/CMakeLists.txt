cmake_minimum_required(VERSION 2.8.3)

project(kdl_parser_py)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS urdfdom_py
)

catkin_python_setup()

catkin_install_python(PROGRAMS ${PROJECT_NAME}/urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_kdl_parser.launch)

  if(PYTHON_EXECUTABLE)
    # Force CMake to find python again in case it was called before to find python 2
    set(old_python_exec "${PYTHON_EXECUTABLE}")
    unset(PYTHON_EXECUTABLE CACHE)
    unset(PYTHONINTERP_FOUND CACHE)
  endif()

  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME test_py3_syntax COMMAND "${PYTHON_EXECUTABLE}" "-m" "flake8" "--select" "E999")

  if(old_python_exec)
    # Restore old executable in case something after expects python 2
    set(PYTHON_EXECUTABLE "${old_python_exec}" CACHE PATH "Python executable found by PythonInterp" FORCE)
  else()
    # Unset in case some catkin script tries to find_package(PythonInterp)
    unset(PYTHON_EXECUTABLE CACHE)
    unset(PYTHONINTERP_FOUND CACHE)
  endif()
endif()
