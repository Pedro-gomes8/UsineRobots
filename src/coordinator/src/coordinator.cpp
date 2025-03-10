#include "coordinator.hpp"
#include "turtle_proxy.hpp"
#include <memory>
#include <thread>
#include <iostream>
#include <cstdio>
#include <functional>

using namespace std;

/**
 * @brief Constructor for the Coordinator node.
 *
 * Initializes the Coordinator node with a "Coordinator" name, creates a reentrant callback group,
 * sets up the arm and turtle services, and sends setup messages.
 */
Coordinator::Coordinator(): Node("Coordinator"){
        // Create a callback group
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // TODO: create a way to track resource ownership
        // TODO: setup interface between coordinator and resource manager

        (void)this->createArmServices();
        (void)this->createTurtleServices();

        RCLCPP_INFO(this->get_logger(), "Service is ready.");
}
/**
 * @brief Sends setup messages.
 *
 * This function is intended to send setup messages during initialization.
 * Currently, it returns -1 as a placeholder.
 *
 * @return int Returns -1 as a temporary placeholder.
 */
int Coordinator::sendSetupMessages(){
    return -1;
}

/**
 * @brief Creates turtle-related services.
 *
 * Sets up the services for handling turtle arrival and initial position notifications.
 * Uses a service quality-of-service profile and registers the callbacks within the service callback group.
 *
 * @return int Returns 0 upon successful creation of services.
 */
int Coordinator::createTurtleServices(){
    //==========================================================================
    //================ Notify Turtle Arrival Service ==========================
    //==========================================================================
    auto qos_profile = rclcpp::QoS(rclcpp::ServicesQoS());

    turtleArrivalService = this->create_service<NotifyTurtleArrival>(
        "notify_turtle_arrival",
        std::bind(&Coordinator::notifyTurtleArrivalCallback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile,
        service_callback_group_
    );

    //==========================================================================
    //==================== Notify Object Initial Position ======================
    //==========================================================================


    turtleInitialPositionService = this->create_service<NotifyTurtleInitialPosition>(
        "notify_turtle_initial_position",
        std::bind(&Coordinator::notifyTurtleInitialPositionCallback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile,
        service_callback_group_
    );

    return 0;
}

/**
 * @brief Creates arm-related services.
 *
 * Sets up the services for handling object movement and arm finished notifications.
 * Uses a service quality-of-service profile and registers the callbacks within the service callback group.
 *
 * @return int Returns 0 upon successful creation of services.
 */
int Coordinator::createArmServices(){
    auto qos_profile = rclcpp::QoS(rclcpp::ServicesQoS());

    //==========================================================================
    //==================== Notify Object Movement Service ======================
    //==========================================================================


    objectMovementService = this->create_service<NotifyObjectMovement>(
        "notify_object_movement",
        std::bind(&Coordinator::notifyObjectMovementCallback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile,
        service_callback_group_
    );

    //==========================================================================
    //==================== Notify Arm Finished =================================
    //==========================================================================

    armFinishedService = this->create_service<NotifyArmFinished>(
        "notify_arm_finished",
        std::bind(&Coordinator::notifyArmFinishedCallback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile,
        service_callback_group_
    );

    return 0;
}

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
void Coordinator::notifyTurtleArrivalCallback(const std::shared_ptr<NotifyTurtleArrival::Request> request,
        std::shared_ptr<NotifyTurtleArrival::Response> response){
    // checking turtle is registered
    std::map<int, TurtleProxy>::iterator it = registeredTurtles.find(request->turtle_id);
    if (it == registeredTurtles.end()) {
        response->ack = -1;
        return;
    }

    // select connecting turtle through request->turtleId
    TurtleProxy turtle = it->second;
    enum TurtlePosition_e newTurtlePosition = static_cast<TurtlePosition_e>(request->turtle_position);

    // updating turtles position
    turtle.changeTurtlePosition(newTurtlePosition);

    // TODO: notify arm of this

    response->ack = 0;
}

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
void Coordinator::notifyTurtleInitialPositionCallback(const std::shared_ptr<NotifyTurtleInitialPosition::Request> request,
        std::shared_ptr<NotifyTurtleInitialPosition::Response> response){
    // Creating a new turtle proxy
    enum TurtlePosition_e newTurtlePosition = static_cast<TurtlePosition_e>(request->turtle_position);
    int newId = -1; // TODO: generate new id
    TurtleProxy newTurtle = TurtleProxy(newId, newTurtlePosition, NO_COLOR, MAX_TURTLE_CAPACITY);

    // include into turtle proxy map
    this->registeredTurtles.insert({newId, newTurtle});

    // respond with turtle's id
    response->turtle_id = newId;
}

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
void Coordinator::notifyObjectMovementCallback(const std::shared_ptr<NotifyObjectMovement::Request> request,
        std::shared_ptr<NotifyObjectMovement::Response> response){
    // checking turtle is registered
    std::map<int, TurtleProxy>::iterator it = registeredTurtles.find(request->turtle_id);
    if (it == registeredTurtles.end()) {
        response->ack = -1;
        return;
    }

    // select connecting turtle through request->turtleId
    TurtleProxy turtle = it->second;

    // if not already done, set color of turtle proxy to request->objectColor
    if(turtle.cargoHasColor(NO_COLOR)){
        turtle.changeCargoType(request->object_color);
    }

    //check if turtle has the same color of the new object
    if(!turtle.cargoHasColor(request->object_color)){
        // if color is different, return an error
        response->ack = -1;
        return;
    }

    (void)turtle.changeCargoAmmount(request->obj_diff);
    if(turtle.isFull() || turtle.isEmpty()){
        turtle.requestCrossing();
        // TODO: take position on the other side resource
        // TODO: determine which position is free
        // TODO: take corridor resource on the other side
        // TODO: send turtle to the OUTPUT side
        response->ack = 1;
    }

    else{
        response->ack = 0;
    }

}

/**
 * @brief Callback for arm finished notifications.
 *
 * This function is invoked when an arm finished event occurs. Currently, it simply acknowledges the event.
 *
 * @param request Shared pointer to the request indicating that the arm has finished its operation.
 * @param response Shared pointer to the response where the acknowledgement (ack) is set.
 */
void Coordinator::notifyArmFinishedCallback(const std::shared_ptr<NotifyArmFinished::Request> request,
                               std::shared_ptr<NotifyArmFinished::Response> response){
    // TODO: what do I do with this information ???

    RCLCPP_INFO(this->get_logger(), "received arm finished request arm_type=%d ammount_moved_objects=%d", request->arm_type, request->ammount_moved_objects);
    response->ack = 0;
}
