cmake_minimum_required(VERSION 3.8)
project(coordinator_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(coordinator_interface
  "srv/NotifyObjectMovement.srv"
  "srv/NotifyTurtleArrival.srv"
  "srv/NotifyTurtleInitialPosition.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the library target and export it.
install(TARGETS
  EXPORT coordinator_interfaceTargets
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files.
install(DIRECTORY include/coordinator_interface/
  DESTINATION include/coordinator_interface
)

# Export the targets so that they can be found by other packages.
ament_export_dependencies(builtin_interfaces)

ament_package()
