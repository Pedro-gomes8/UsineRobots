cmake_minimum_required(VERSION 3.8)
project(turtle_coordinator_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(turtle_coordinator_interface
  "srv/NotifyTurtleArrival.srv"
  "srv/NotifyTurtleInitialPosition.srv"
  "msg/TurtleMove.msg"
  DEPENDENCIES
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the library target
add_library(turtle_coordinator_interface_lib
  src/turtle.c
)

# Use target_include_directories to specify public include paths.
target_include_directories(turtle_coordinator_interface_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/turtle_coordinator_interface>
  $<INSTALL_INTERFACE:include/turtle_coordinator_interface>
)

# Install the library target and export it.
install(TARGETS
  turtle_coordinator_interface_lib
  EXPORT turtle_coordinator_interfaceTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files.
install(DIRECTORY include/turtle_coordinator_interface/
  DESTINATION include/turtle_coordinator_interface
)

# Export the targets so that they can be found by other packages.
ament_export_targets(turtle_coordinator_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
