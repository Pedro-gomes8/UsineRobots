cmake_minimum_required(VERSION 3.8)
project(coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(arm_coordinator_interface REQUIRED)
find_package(turtle_interface REQUIRED)
find_package(resource_manager_interface REQUIRED)
find_package(coordinator_interface REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include/coordinator/)

# Rename the executable target to avoid conflict with the generated interface target
add_executable(coordinator_node
  src/coordinator.cpp
  src/coordinator_main.cpp
  src/turtle_proxy.cpp
  src/arm_proxy.cpp
  src/resource_manager_proxy.cpp
  )

ament_target_dependencies(coordinator_node coordinator_interface resource_manager_interface arm_coordinator_interface turtle_interface rclcpp std_msgs)

target_link_libraries(coordinator_node turtle_interface::turtle_interface_lib)
target_link_libraries(coordinator_node arm_coordinator_interface::arm_coordinator_interface_lib)
target_link_libraries(coordinator_node resource_manager_interface::resource_manager_interface_lib)
target_link_libraries(coordinator_node rosidl_typesupport_cpp::rosidl_typesupport_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  coordinator_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
