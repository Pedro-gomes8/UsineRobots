#include "coordinator.hpp"
#include "turtle_proxy.hpp"
#include <memory>
#include <thread>
#include <iostream>
#include <cstdio>
#include <functional>


using namespace std;

void notifyTurtleArrivalCallback(const std::shared_ptr<NotifyTurtleArrival::Request> request,
        std::shared_ptr<NotifyTurtleArrival::Response> response);

void notifyTurtleInitialPositionCallback(const std::shared_ptr<NotifyTurtleInitialPosition::Request> request,
        std::shared_ptr<NotifyTurtleInitialPosition::Response> response);

void notifyObjectMovementCallback(const std::shared_ptr<NotifyObjectMovement::Request> request,
        std::shared_ptr<NotifyObjectMovement::Response> response);

void notifyArmFinishedCallback(const std::shared_ptr<NotifyArmFinished::Request> request,
        std::shared_ptr<NotifyArmFinished::Response> response);

Coordinator::Coordinator(): Node("Coordinator"){
        // Create a callback group
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        (void)this->createArmServices();
        (void)this->createTurtleServices();
        (void)this->sendSetupMessages();

        RCLCPP_INFO(this->get_logger(), "Service is ready.");
}

int Coordinator::sendSetupMessages(){
    return -1;
}

int Coordinator::createTurtleServices(){
    //==========================================================================
    //================ Notify Turtle Arrival Service ==========================
    //==========================================================================
    auto qos_profile = rclcpp::QoS(rclcpp::ServicesQoS());

    turtleArrivalService = this->create_service<NotifyTurtleArrival>(
        "notify_turtle_arrival",
        notifyTurtleArrivalCallback,
        qos_profile,
        service_callback_group_
    );

    //==========================================================================
    //==================== Notify Object Initial Position ======================
    //==========================================================================


    turtleInitialPositionService = this->create_service<NotifyTurtleInitialPosition>(
        "notify_turtle_initial_position",
        notifyTurtleInitialPositionCallback,
        qos_profile,
        service_callback_group_
    );

    return 0;
}

int Coordinator::createArmServices(){
    auto qos_profile = rclcpp::QoS(rclcpp::ServicesQoS());

    //==========================================================================
    //==================== Notify Object Movement Service ======================
    //==========================================================================


    objectMovementService = this->create_service<NotifyObjectMovement>(
        "notify_object_movement",
        notifyObjectMovementCallback,
        qos_profile,
        service_callback_group_
    );

    //==========================================================================
    //==================== Notify Arm Finished =================================
    //==========================================================================

    armFinishedService = this->create_service<NotifyArmFinished>(
        "notify_arm_finished",
        notifyArmFinishedCallback,
        qos_profile,
        service_callback_group_
    );

    return 0;
}

void notifyTurtleArrivalCallback(const std::shared_ptr<NotifyTurtleArrival::Request> request,
        std::shared_ptr<NotifyTurtleArrival::Response> response){
    // TODO: select connecting turtle through request->turtleId
    TurtleProxy turtle = this->registeredTurtles.find(request->turtleId);
    enum TurtlePosition_e newTurtlePosition = static_cast<TurtlePosition_e>(request->turtlePosition);

    // updating turtles position
    turtle->changeTurtlePosition(newTurtlePosition);

    response->ack = 0;
}

void notifyTurtleInitialPositionCallback(const std::shared_ptr<NotifyTurtleInitialPosition::Request> request,
        std::shared_ptr<NotifyTurtleInitialPosition::Response> response){
    // Creating a new turtle proxy
    enum TurtlePosition_e newTurtlePosition = static_cast<TurtlePosition_e>(request->turtlePosition);
    int newId = -1; // TODO: generate new id
    TurtleProxy newTurtle = TurtleProxy(newTurtlePosition, NO_COLOR, MAX_TURTLE_CAPACITY);

    // TODO: include into turtle proxy map
    this->registeredTurtles.insert({newId, newTurtle});

    // respond with turtle's id
    response->turtleId = newId;

}

void notifyObjectMovementCallback(const std::shared_ptr<NotifyObjectMovement::Request> request,
        std::shared_ptr<NotifyObjectMovement::Response> response){
    TurtleProxy turtle = this->registeredTurtles.find(request->turtleId);

    // if not already done, set color of turtle proxy to request->objectColor
    if(turtle.cargoHasColor(NO_COLOR)){
        turtle.changeCargoType(request->objectColor);
    }

    //check if turtle has the same color of the new object
    if(!turtle.cargoHasColor(request->objectColor)){
        // if color is different, return an error
        response->ack = -1;
        return;
    }

    turtle.changeCargoAmmount(request->objDiff);
    if(turtle.isFull()){
        //TODO: send to the OUTPUT side
        response->ack = 1;
    }

    else if(turtle.isEmpty()){
        //TODO: send to the INPUT side
        response->ack = 1;
    }

    else{
        response->ack = 0;
    }

}

void notifyArmFinishedCallback(const std::shared_ptr<NotifyArmFinished::Request> request,
                               std::shared_ptr<NotifyArmFinished::Response> response){
    // TODO: what do I do with this information ???
    response->ack = 0;
}
