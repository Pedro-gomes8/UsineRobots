cmake_minimum_required(VERSION 3.8)
project(resource_manager_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This generates an interface target named "resource_manager"
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestResource.srv"
  DEPENDENCIES std_msgs
)

include_directories(include/resource_manager_interface/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the library target
add_library(resource_manager_interface_lib
  src/resource_manager_interface.cpp
)

# Use target_include_directories to specify public include paths.
target_include_directories(resource_manager_interface_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/resource_manager_interface>
  $<INSTALL_INTERFACE:include/resource_manager_interface>
)

# Install the library target and export it.
install(TARGETS
  resource_manager_interface_lib
  EXPORT resource_manager_interfaceTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files.
install(DIRECTORY include/resource_manager_interface/
  DESTINATION include/resource_manager_interface
)

# Export the targets so that they can be found by other packages.
ament_export_targets(resource_manager_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
