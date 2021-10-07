cmake_minimum_required(VERSION 3.5)
project(pacmod_game_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(pacmod3_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Build

add_executable(${PROJECT_NAME}_node
  src/pacmod_game_control_exec.cpp
  src/pacmod_game_control.cpp
  src/startup_checks.cpp
  src/controllers.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "pacmod3_msgs"
  "sensor_msgs"
  "std_msgs"
)

ament_export_include_directories(include)

# Install

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
