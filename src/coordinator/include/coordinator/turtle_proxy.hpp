/**
 * @file turtle_proxy.h
 * @brief Declaration of the TurtleProxy class.
 *
 * This header file defines the TurtleProxy class which manages the state and operations
 * of a turtle entity including its position, cargo amount, and cargo type.
 *
 */
#ifndef TURTLE_PROXY_H_
#define TURTLE_PROXY_H_
#include "turtle.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

using namespace std;

/**
 * @class TurtleProxy
 * @brief Proxy class for managing turtle operations.
 *
 * The TurtleProxy class encapsulates the properties and operations for a turtle, including
 * its current position, cargo content (color), and cargo capacity. It provides methods to
 * change the turtle's position, update its cargo amount and type, and check if the turtle
 * is full or empty.
 */
class TurtleProxy{
  public:
    /**
     * @brief Constructor for TurtleProxy.
     *
     * Initializes a new TurtleProxy with the given position, cargo color, and cargo limit.
     *
     * @param turtleId The Id the turtle got from the coordinator.
     * @param position The initial position of the turtle.
     * @param contentColor The initial cargo color of the turtle.
     * @param cargoLimit The maximum capacity of the turtle's cargo.
     * @param node The pointer to the node that's using this proxy
     */
    TurtleProxy(int turtleId,TurtlePosition_e position, string contentColor, int cargoLimit,shared_ptr<rclcpp::Node> node);

    /**
     * @brief Changes the turtle's position.
     *
     * Updates the turtle's position to the new specified value.
     *
     * @param newPosition The new position for the turtle.
     * @return int Status code indicating success (typically 0) or an error code.
     */
    int changeTurtlePosition(TurtlePosition_e newPosition);

    /**
     * @brief Checks if the turtle's cargo is full.
     *
     * Determines if the current cargo amount has reached the turtle's cargo limit.
     *
     * @return bool True if the cargo amount equals the cargo limit, false otherwise.
     */
    bool isFull();

    /**
     * @brief Checks if the turtle's cargo is empty.
     *
     * Determines if the current cargo amount is zero.
     *
     * @return bool True if the cargo amount is zero, false otherwise.
     */
    bool isEmpty();

    /**
     * @brief Checks if the turtle's cargo has the specified color.
     *
     * Compares the current cargo color with the provided reference color.
     *
     * @param referenceColor The color to be compared with the turtle's cargo color.
     * @return bool True if the cargo color matches the reference color, false otherwise.
     */
    bool cargoHasColor(string referenceColor);

    string getCargoColor();

    /**
     * @brief Changes the turtle's cargo amount.
     *
     * Modifies the current cargo amount by the given value. The value can be positive or negative.
     *
     * @param ammount The amount to adjust the cargo by.
     * @return int The updated cargo amount.
     */
    int changeCargoAmmount(int ammount);

    /**
     * @brief Changes the turtle's cargo type.
     *
     * Updates the turtle's cargo color to the new specified cargo color.
     *
     * @param cargoColor The new cargo color.
     * @return int Status code indicating success (typically 0) or an error code.
     */
    int changeCargoType(string cargoColor);


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
     * arrived on the otherside to set the new position
     */
    int requestCrossing(TurtlePosition_e position);

    /**
    * @brief Return a position that is on the given list that is not occupied by any
    * of the registered turtles
    *
    * This function is invoked whenever a turtle has to switch sides. When a turtle
    * takes a resource of a place in one of the sides, it's also necessary to
    * determine which of the sides it has locked
    * @param desiredPositions The list of positions that'll be filtered
    */
    TurtlePosition_e getPosition();
  private:
    int turtleId;
    int cargoAmmount;
    TurtlePosition_e position;
    string contentColor;
    int cargoLimit;
    shared_ptr<rclcpp::Node> node;

};

#endif // TURTLE_PROXY_H_
