cmake_minimum_required(VERSION 3.8)
project(py_armnode_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(arm_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(py_armnode)

#ament_export_dependencies(
#  rosidl_default_runtime
#)

install(DIRECTORY 
  py_armnode/
  #py_armnode/armnode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS 
  py_armnode/armnode.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

