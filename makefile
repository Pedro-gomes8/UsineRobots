##
# Project Title
#
# @file
# @version 0.1

coordinator:
	colcon build --packages-select coordinator
arm_coordinator_interface:
	colcon build --packages-select arm_coordinator_interface
turtle_coordinator_interface:
	colcon build --packages-select turtle_coordinator_interface
resource_manager_interface:
	colcon build --packages-select resource_manager_interface
resource_manager:
	colcon build --packages-select resource_manager


# end
