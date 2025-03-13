##
# Project Title
#
# @file
# @version 0.1

interfaces: coordinator_interface arm_interface turtle_interface resource_manager_interface

nodes: coordinator resource_manager

run_coordinator:
	ros2 run resource_manager resource_manager_node &
	ros2 run coordinator coordinator_node

coordinator:
	colcon build --packages-select coordinator

coordinator_interface:
	colcon build --packages-select coordinator_interface

arm_interface:
	colcon build --packages-select arm_interface

turtle_interface:
	colcon build --packages-select turtle_interface

resource_manager_interface:
	colcon build --packages-select resource_manager_interface

resource_manager:
	colcon build --packages-select resource_manager


# end
