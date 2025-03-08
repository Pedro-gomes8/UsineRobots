cmake_minimum_required(VERSION 3.8)
project(resource_manager)

# Default to C++14

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include/resource_manager/)

add_executable(resource_manager src/resource_manager.cpp src/resource_database.cpp src/resource_database_proxy.cpp )
ament_target_dependencies(resource_manager rclcpp std_msgs)

install(TARGETS
  resource_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()
