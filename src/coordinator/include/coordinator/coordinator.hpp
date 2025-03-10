#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include "rclcpp/rclcpp.hpp"
#include <list>
#include "turtle_proxy.hpp"

#include "turtle_coordinator_interface/srv/notify_turtle_arrival.hpp"
#include "turtle_coordinator_interface/srv/notify_turtle_initial_position.hpp"
#include "turtle_coordinator_interface/msg/turtle_move.hpp"

#include "arm_coordinator_interface/srv/notify_object_movement.hpp"
#include "arm_coordinator_interface/srv/notify_arm_finished.hpp"
#include "arm_coordinator_interface/srv/input_arm_setup.hpp"

#define MAX_TURTLE_CAPACITY 5
#define NO_COLOR "None"

using NotifyObjectMovement = arm_coordinator_interface::srv::NotifyObjectMovement;
using NotifyArmFinished = arm_coordinator_interface::srv::NotifyArmFinished;
using InputArmSetup = arm_coordinator_interface::srv::InputArmSetup;

using NotifyTurtleArrival = turtle_coordinator_interface::srv::NotifyTurtleArrival;
using NotifyTurtleInitialPosition = turtle_coordinator_interface::srv::NotifyTurtleInitialPosition;
using TurtleMove = turtle_coordinator_interface::msg::TurtleMove;

class Coordinator : public rclcpp::Node{
  public:
    Coordinator() ;
    int sendSetupMessages();

    int createTurtleServices();

    int createArmServices();

  private:
    map<int,TurtleProxy> registeredTurtles;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    rclcpp::Service<NotifyTurtleArrival>::SharedPtr turtleArrivalService;
    rclcpp::Service<NotifyTurtleInitialPosition>::SharedPtr turtleInitialPositionService;
    rclcpp::Service<NotifyObjectMovement>::SharedPtr objectMovementService;
    rclcpp::Service<NotifyArmFinished>::SharedPtr armFinishedService;
};




#endif // COORDINATOR_H_
