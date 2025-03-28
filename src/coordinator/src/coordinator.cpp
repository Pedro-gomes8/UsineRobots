// dependencies from the current package
#include "coordinator.hpp"
#include "turtle_proxy.hpp"
#include "resource_manager_proxy.hpp"
#include "resource_manager_interface.hpp"
#include "arm_proxy.hpp"

// dependencies from other project packages
#include "arm_interface.hpp"

// outside dependencies
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
Coordinator::Coordinator(): Node("Coordinator"), inputArm(INPUT_ARM), outputArm(OUTPUT_ARM){
        // Create a callback group
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        (void)this->createArmServices();
        (void)this->createTurtleServices();

        this->validId = 0;

        RCLCPP_INFO(this->get_logger(), "Service is ready.");
}

/**
 * @brief Function to initialize proxies
 *
 * Initializes the proxies used by the coordinator
 */
int Coordinator::init(){
    (void)this->resourceManager.init(shared_from_this());
    (void)this->inputArm.init(shared_from_this());
    (void)this->outputArm.init(shared_from_this());
    return 0;
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
    int turtleId = request->turtle_id;
    enum TurtlePosition_e newTurtlePosition = static_cast<TurtlePosition_e>(request->turtle_position);
    if (it == registeredTurtles.end()) {
        response->ack = -1;
        return;
    }

    // select connecting turtle through request->turtleId
    TurtleProxy turtle = it->second;

    // releasing resources
    if(turtle.getPosition() == INPUT_SIDE1 || turtle.getPosition() == INPUT_SIDE2){

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing place resource of new Turtle");
        (void)this->resourceManager.releaseResource(RESOURCE_INPUT_SIDE);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Input Arm about it");
        (void)this->inputArm.notifyTurtleArrived(turtleId,turtle.getCargoColor(),request->x_turtle,request->y_turtle);
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing place resource of new Turtle");
        (void)this->resourceManager.releaseResource(RESOURCE_OUTPUT_SIDE);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Input Arm about it");
        (void)this->outputArm.notifyTurtleArrived(turtleId,turtle.getCargoColor(), request->x_turtle,request->y_turtle);
    }

    (void)this->resourceManager.releaseResource(RESOURCE_CORRIDOR);


    // updating turtles position
    turtle.changeTurtlePosition(newTurtlePosition);

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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting up new Turtle");
    enum TurtlePosition_e newTurtlePosition = static_cast<TurtlePosition_e>(request->turtle_position);
    int newId = this->getNewValidId();
    TurtleProxy newTurtle = TurtleProxy(newId, newTurtlePosition, NO_COLOR,
                            MAX_TURTLE_CAPACITY, shared_from_this());

    // lock respective resource
    if(newTurtlePosition == INPUT_SIDE1 || newTurtlePosition == INPUT_SIDE2){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Locking place resource of new Turtle");
        (void)this->resourceManager.lockResource(RESOURCE_INPUT_SIDE);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Input Arm about it");
        (void)this->inputArm.notifyTurtleArrived(newId,NO_COLOR,request->x_turtle,request->y_turtle);
    }

    else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Locking place resource of new Turtle");
        (void)this->resourceManager.lockResource(RESOURCE_OUTPUT_SIDE);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Input Arm about it");
        (void)this->outputArm.notifyTurtleArrived(newId,NO_COLOR,request->x_turtle,request->y_turtle);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Registering new Turtle");
    // include into turtle proxy map
    this->registeredTurtles.insert({newId, newTurtle});


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finished initial seting up turtle %d", newId);
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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking Turtle Has a Color");
    // if not already done, set color of turtle proxy to request->objectColor
    if(turtle.cargoHasColor(NO_COLOR)){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting Turtle Color");
        turtle.changeCargoType(request->object_color);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating ammount on the turtle");
    (void)turtle.changeCargoAmmount(request->obj_diff);
    if(turtle.isFull()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turtle is Full");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting Place Resource on the OUTPUT SIDE");
        (void)this->resourceManager.lockResource(RESOURCE_OUTPUT_SIDE);
        TurtlePosition_e destination = this->getAvailablePosition({OUTPUT_SIDE1, OUTPUT_SIDE2});

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting Corridor Resource");
        (void)this->resourceManager.lockResource(RESOURCE_CORRIDOR);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Turtle to Cross");
        turtle.requestCrossing(destination);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Arm the Turtle is gone");
        response->ack = 1;
    }

    else if(turtle.isEmpty()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turtle is Empty");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting Place Resource on the OUTPUT SIDE");
        (void)this->resourceManager.lockResource(RESOURCE_INPUT_SIDE);
        TurtlePosition_e destination = this->getAvailablePosition({INPUT_SIDE1, INPUT_SIDE2});

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting Corridor Resource");
        (void)this->resourceManager.lockResource(RESOURCE_CORRIDOR);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Turtle to Cross");
        turtle.requestCrossing(destination);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Telling Arm the Turtle is gone");
        response->ack = 1;
    }

    else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACK to the ARM");
        response->ack = 0;
    }

}

/**
* @brief Function that generates a unique id to assign to a connecting element
*
* This function is invoked whenever a new turtle connects to the coordinator
*/
int Coordinator::getNewValidId(){
    this->validId +=1;
    return this->validId;
}

/**
* @brief Return a position that is on the given list that is not occupied by any
* of the registered turtles
*
* This function is invoked whenever a turtle has to switch sides. When a turtle
* takes a resource of a place in one of the sides, it's also necessary to
* determine which of the sides it has locked
* @param desiredPositions The list of positions that'll be filtered
*/
TurtlePosition_e Coordinator::getAvailablePosition(vector<TurtlePosition_e> desiredPositions){
    //TODO: unit test this
    unordered_set<TurtlePosition_e> occupiedPositions;

    // Add occupied positions from each TurtleProxy to the set.
    for (auto& entry : this->registeredTurtles) {
        occupiedPositions.insert(entry.second.getPosition());
    }

    // Collect positions from newPositions that are not already occupied.
    vector<TurtlePosition_e> available;
    for (const TurtlePosition_e& pos : desiredPositions) {
        if (occupiedPositions.find(pos) == occupiedPositions.end()) {
            available.push_back(pos);
        }
    }

    return available[0];
}
