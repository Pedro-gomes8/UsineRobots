##
# Project Title
#
# @file
# @version 0.1

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
