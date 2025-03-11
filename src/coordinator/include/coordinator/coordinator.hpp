/**
 * @file coordinator.hpp
 * @brief class to define the coordinator object.
 *
 * This module defines all the functions to setup the coordinator
 */
#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include "rclcpp/rclcpp.hpp"
#include <list>
#include "turtle_proxy.hpp"
#include "arm_proxy.hpp"
#include "resource_manager_proxy.hpp"

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


/**
 * @brief This class manages the coordination of turtle services and their integration with the system.
 */
class Coordinator : public rclcpp::Node{
  public:
    Coordinator();

    int init();

    enum TurtlePosition_e getFreePositionFromResource();

  private:
    /**
     * @brief Creates all necessary turtle services.
     * @details This method creates the services required for the turtles to operate.
    */
    int createTurtleServices();
    /**
     * @brief Creates all necessary arm services.
     * @details This method creates the services required for the arms to operate.
    */
    int createArmServices();

    /**
    * @brief Callback for turtle arrival notifications.
    *
    * This function is invoked when a turtle arrives. It retrieves the corresponding turtle proxy from
    * the registered turtles using the turtleId in the request, updates its position based on the request,
    * and acknowledges the request.
    *
    * @param request Shared pointer to the request containing the turtleId and new turtle position.
    * @param response Shared pointer to the response where the acknowledgement (ack) is set.
    */
    void notifyTurtleArrivalCallback(const std::shared_ptr<NotifyTurtleArrival::Request> request,
            std::shared_ptr<NotifyTurtleArrival::Response> response);
    /**
    * @brief Callback for initial turtle position notifications.
    *
    * This function is invoked when a new turtle is initialized. It creates a new TurtleProxy with the
    * specified initial position, assigns a new turtle id (currently a placeholder), registers the turtle,
    * and responds with the assigned turtle id.
    *
    * @param request Shared pointer to the request containing the initial turtle position.
    * @param response Shared pointer to the response where the new turtle's id is returned.
    */
    void notifyTurtleInitialPositionCallback(const std::shared_ptr<NotifyTurtleInitialPosition::Request> request,
            std::shared_ptr<NotifyTurtleInitialPosition::Response> response);
    /**
    * @brief Callback for object movement notifications.
    *
    * This function processes movement requests for objects. It retrieves the turtle proxy corresponding
    * to the turtleId from the request, sets the cargo color if not already set, and updates the cargo amount.
    * Depending on whether the turtle becomes full or empty, additional actions are suggested via TODO comments.
    *
    * @param request Shared pointer to the request containing object movement details such as turtleId,
    *                object color, and difference in object amount.
    * @param response Shared pointer to the response where the acknowledgement (ack) is set based on the update.
    */
    void notifyObjectMovementCallback(const std::shared_ptr<NotifyObjectMovement::Request> request,
            std::shared_ptr<NotifyObjectMovement::Response> response);
    /**
    * @brief Callback for arm finished notifications.
    *
    * This function is invoked when an arm finished event occurs. Currently, it simply acknowledges the event.
    *
    * @param request Shared pointer to the request indicating that the arm has finished its operation.
    * @param response Shared pointer to the response where the acknowledgement (ack) is set.
    */
    void notifyArmFinishedCallback(const std::shared_ptr<NotifyArmFinished::Request> request,
            std::shared_ptr<NotifyArmFinished::Response> response);

    int getNewValidId();

    TurtlePosition_e getAvailablePosition(vector<TurtlePosition_e> desiredPositions);

    /**
     * @var The map of registered turtles.
     * @details This map stores all the turtles that are registered with the
     * system through the turtleInitialPosition service
    */
    map<int,TurtleProxy> registeredTurtles;
    int validId;

    ArmProxy inputArm;
    ArmProxy outputArm;

    ResourceManagerProxy resourceManager;


    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    rclcpp::Service<NotifyTurtleArrival>::SharedPtr turtleArrivalService;
    rclcpp::Service<NotifyTurtleInitialPosition>::SharedPtr turtleInitialPositionService;
    rclcpp::Service<NotifyObjectMovement>::SharedPtr objectMovementService;
    rclcpp::Service<NotifyArmFinished>::SharedPtr armFinishedService;
};




#endif // COORDINATOR_H_
