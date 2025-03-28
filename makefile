##
# Project Title
#
# @file
# @version 0.1
interfaces: coordinator_interface arm_interface turtle_interface resource_manager_interface custom_action_interfaces
	@printf "Remember to run the command:\n"
	@printf "source install/setup.bash\n"

nodes: coordinator resource_manager custom_action_cpp

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

custom_action_interfaces:
	colcon build --packages-select custom_action_interfaces

custom_action_cpp:
	colcon build --packages-select custom_action_cpp

clean:
	rm -fr build log install
