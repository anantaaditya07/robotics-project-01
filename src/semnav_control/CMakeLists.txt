cmake_minimum_required(VERSION 3.8)
project(semnav_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY include/ DESTINATION include)

add_executable(safety_gate_node src/safety_gate_node.cpp)
target_include_directories(safety_gate_node PRIVATE include)
ament_target_dependencies(safety_gate_node geometry_msgs rclcpp sensor_msgs)
install(TARGETS safety_gate_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Gate logic (architecture 7.4, D-13): plain STL, no ROS.
  ament_add_gtest(test_safety_math test/test_safety_math.cpp)
  target_include_directories(test_safety_math PRIVATE include)
endif()

ament_package()
