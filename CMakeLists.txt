cmake_minimum_required(VERSION 3.8)
project(pidcontroll_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(intelligent_humanoid_interfaces REQUIRED)

include_directories(
    include
  )

add_executable(pidcontroll_node
  src/pidcontroll_pkg.cpp
  include/PID_controller/pidcontroll.cpp
)

ament_target_dependencies(pidcontroll_node
  rclcpp
  intelligent_humanoid_interfaces
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS
  pidcontroll_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include)

# Install launch & config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
