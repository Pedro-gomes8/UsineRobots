/**
 * @file turtle_proxy.cpp
 * @brief Implementation of the TurtleProxy class methods.
 *
 * This file provides the definitions for the TurtleProxy class which manages the state and
 * operations of a turtle, including its position, cargo amount, and cargo type.
 *
 * @note NOT THREAD SAFE
 */

#include "turtle_proxy.hpp"
#include <memory>
#include <string>

using namespace std;

/**
 * @brief Constructor for TurtleProxy.
 *
 * Initializes a new TurtleProxy instance with the given position, cargo color, and cargo limit.
 * The cargo amount is initialized to zero.
 *
 * @param position The initial position of the turtle.
 * @param contentColor The initial cargo color.
 * @param cargoLimit The maximum cargo capacity.
 * @param node The pointer to the node that's using this proxy
 */
TurtleProxy::TurtleProxy(int turtleId,TurtlePosition_e position, string contentColor, int cargoLimit, shared_ptr<rclcpp::Node> node)
    : turtleId(turtleId),position(position), contentColor(contentColor), cargoLimit(cargoLimit), cargoAmmount(0), node(node)
{
    this->client = node->create_client<TurtleMove>("TurtleMove"+to_string(this->turtleId));
}

/**
 * @brief Changes the turtle's position.
 *
 * Updates the turtle's current position to the new specified position.
 *
 * @param newPosition The new position for the turtle.
 * @return int Returns 0 to indicate success.
 */
int TurtleProxy::changeTurtlePosition(TurtlePosition_e newPosition) {
    position = newPosition;
    return 0;
}

/**
 * @brief Checks if the turtle's cargo is full.
 *
 * Determines whether the current cargo amount has reached the cargo limit.
 *
 * @return bool True if the cargo amount equals the cargo limit, false otherwise.
 */
bool TurtleProxy::isFull() {
    return cargoAmmount == cargoLimit;
}

/**
 * @brief Checks if the turtle's cargo is empty.
 *
 * Determines whether the current cargo amount is zero.
 *
 * @return bool True if the cargo amount is zero, false otherwise.
 */
bool TurtleProxy::isEmpty() {
    return cargoAmmount == 0;
}

/**
 * @brief Checks if the turtle's cargo has the specified color.
 *
 * Compares the current cargo color with the provided reference color.
 *
 * @param referenceColor The color to be compared with the turtle's cargo color.
 * @return bool True if the cargo color matches the reference color, false otherwise.
 */
bool TurtleProxy::cargoHasColor(string referenceColor) {
    return contentColor == referenceColor;
}

/**
 * @brief Changes the turtle's cargo amount.
 *
 * Adjusts the current cargo amount by the given value. If the operation would result in an
 * invalid cargo amount (below 0 or above cargoLimit), the change is not applied and an error code
 * is returned.
 *
 * @param ammount The amount to change the cargo by (can be negative).
 * @return int Returns 0 on success, or -1 if the operation would exceed valid boundaries.
 */
int TurtleProxy::changeCargoAmmount(int ammount) {
    int newAmount = cargoAmmount + ammount;
    if (newAmount < 0 || newAmount > cargoLimit) {
        return -1; // Error: invalid operation.
    }
    cargoAmmount = newAmount;
    return 0; // Success.
}

/**
 * @brief Changes the turtle's cargo type.
 *
 * Updates the cargo type (color) of the turtle.
 *
 * @param cargoColor The new cargo color.
 * @return int Returns 0 to indicate success.
 */
int TurtleProxy::changeCargoType(string cargoColor) {
    contentColor = cargoColor;
    return 0;
}

string TurtleProxy::getCargoColor(){
    return this->contentColor;
}
/**
 * @brief asks the turtle to cross to the other side.
 *
 * utilizes a ros2 service to ask the specific turtle node to move to the
 * one of the positions on the side opposite to the one it currently is.  To
 * ensure no collisions happen, this also implies asking for the resource
 * manager for the corridor and place ressources as per described in the
 * projects PT network diagram
 *
 *
 * @return int Status code indicating success (typically 0) or an error code.
 * @note the notifyTurtleArrival service will be used once the turtle has
 * arrived on the otherside to set the new position, in the meanwhile, the
 * turtleProxy's position is not known
 */
int TurtleProxy::requestCrossing(TurtlePosition_e position){

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "creating request object to notify arm");
  shared_ptr<TurtleMove::Request> req = make_shared<TurtleMove::Request>();
  req->turtle_id = turtleId;
  req->turtle_position= position;

  while (!this->client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request sent to arm robot");
  auto result = client->async_send_request(req);

  return 0;
}


TurtlePosition_e TurtleProxy::getPosition(){
    return this->position;
}
