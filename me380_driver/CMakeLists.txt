cmake_minimum_required(VERSION 3.8)
project(me380_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(driver_node src/driver_node.cpp)
ament_target_dependencies(driver_node rclcpp sensor_msgs std_msgs)

install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/driver_node_py.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
