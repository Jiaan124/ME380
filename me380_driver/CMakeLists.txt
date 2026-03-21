cmake_minimum_required(VERSION 3.8)
project(me380_driver)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/driver_node_py.py
  scripts/ik.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
