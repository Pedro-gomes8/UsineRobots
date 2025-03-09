#include "coordinator.h"

#include "arm_coordinator_interface/srv/NotifyObjectMovement"
#include "arm_coordinator_interface/srv/NotifyArmFinished"
#include "arm_coordinator_interface/srv/NotifyTurtleStateChange"
#include "arm_coordinator_interface/srv/InputArmSetup"

#include "turtle_coordinator_interface/srv/NotifyTurtleArrival"
#include "turtle_coordinator_interface/srv/NotifyTurtleInitialPosition"
#include "turtle_coordinator_interface/msg/TurtleMove"

using NotifyObjectMovement = arm_coordinator_interface::srv::NotifyObjectMovement;
using NotifyArmFinished = arm_coordinator_interface::srv::NotifyArmFinished;
using NotifyTurtleStateChange = arm_coordinator_interface::srv::NotifyTurtleStateChange;
using InputArmSetup = arm_coordinator_interface::srv::InputArmSetup;

using NotifyTurtleArrival = turtle_coordinator_interface::srv::NotifyTurtleArrival;
using NotifyTurtleInitialPosition = turtle_coordinator_interface::srv::NotifyTurtleInitialPosition;
using TurtleMove = turtle_coordinator_interface::msg::TurtleMove;

int Coordinator::sendSetupMessages(shared_ptr<rclcpp::Node> node, Coordinator coord){
    return -1;
}

int Coordinator::createTurtleServices(shared_ptr<rclcpp::Node> node, Coordinator coord){
    //==========================================================================
    //================ Notify Turtle Arrival Service ==========================
    //==========================================================================
    auto turtleArrivalCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto qos_profile = rclcpp::QoS(rclcpp::ServicesQoS());

    auto turtleArrivalService = node->create_service<NotifyTurtleArrival>(
        "notify_turtle_arrival",
        bind(notifyTurtleArrivalCallback,placeholders::_1,placeholders::_2,placeholders::_3,coordinator),
        qos_profile,
        turtleArrivalCallbackGroup
    );

    //==========================================================================
    //==================== Notify Object Initial Position ======================
    //==========================================================================


    auto turtleInitialPositionCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto turtleInitialPositionService = node->create_service<NotifyObjectMovement>(
        "notify_turtle_initial_position",
        bind(notifyTurtleInitialPositionCallback,placeholders::_1,placeholders::_2,placeholders::_3,coordinator),
        qos_profile,
        turtleInitialPositionCallbackGroup
    );

    return 0;
}

int Coordinator::createArmServices(shared_ptr<rclcpp::Node> node, Coordinator coord){

    //==========================================================================
    //================ Notify Turtle State Change Service ======================
    //==========================================================================
    auto turtleStateChangeCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto qos_profile = rclcpp::QoS(rclcpp::ServicesQoS());

    auto turtleStateChangeService = node->create_service<NotifyTurtleStateChange>(
        "notify_turtle_state_change",
        bind(notifyTurtleStateChangeCallback,placeholders::_1,placeholders::_2,placeholders::_3,coordinator),
        qos_profile,
        turtleStateChangeCallbackGroup
    );

    //==========================================================================
    //==================== Notify Object Movement Service ======================
    //==========================================================================


    auto objectMovementCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto objectMovementService = node->create_service<NotifyObjectMovement>(
        "notify_object_movement",
        bind(notifyObjectMovementCallback,placeholders::_1,placeholders::_2,placeholders::_3,coordinator),
        qos_profile,
        objectMovementCallbackGroup
    );

    //==========================================================================
    //==================== Notify Arm Finished =================================
    //==========================================================================

    auto armFinishedCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto armFinishedService = node->create_service<NotifyArmFinished>(
        "notify_arm_finished",
        bind(notifyArmFinishedCallback,placeholders::_1,placeholders::_2,placeholders::_3,coordinator),
        qos_profile,
        armFinishedCallbackGroup
    );

    return 0;
}
