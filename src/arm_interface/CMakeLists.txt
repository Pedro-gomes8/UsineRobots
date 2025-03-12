cmake_minimum_required(VERSION 3.8)
project(arm_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(arm__interface
  "srv/TurtleBotArrived.srv"
)

include_directories(include/arm_interface/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#
# Create the library target
add_library(arm_interface_lib
  src/arm_interface.cpp
)

# Use target_include_directories to specify public include paths.
target_include_directories(arm_interface_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/arm_interface>
  $<INSTALL_INTERFACE:include/arm_interface>
)

# Install the library target and export it.
install(TARGETS
  arm_interface_lib
  EXPORT arm_interfaceTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files.
install(DIRECTORY include/arm_interface/
  DESTINATION include/arm_interface
)

# Export the targets so that they can be found by other packages.
ament_export_targets(arm_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(builtin_interfaces)

ament_package()
