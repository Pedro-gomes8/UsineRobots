cmake_minimum_required(VERSION 3.8)
project(resource_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(resource_manager_interface REQUIRED)

include_directories(include/resource_manager/)

# Rename the executable target to avoid conflict with the generated interface target
add_executable(resource_manager_node
  src/resource_manager.cpp
  src/resource_database.cpp
  src/resource_database_proxy.cpp
  src/resource_request_respond.cpp)
ament_target_dependencies(resource_manager_node resource_manager_interface rclcpp std_msgs)
target_link_libraries(resource_manager_node resource_manager_interface::resource_manager_interface_lib)

install(TARGETS
  resource_manager_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
