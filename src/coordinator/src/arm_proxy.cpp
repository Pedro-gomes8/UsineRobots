/**
 * @file arm_proxy.cpp
 * @brief Implementation of the ArmProxy class.
 *
 * This file provides the method definitions for the ArmProxy class,
 * which serves as an interface for managing robotic arms in a ROS2 environment.
 */
#include "rclcpp/rclcpp.hpp"
#include "arm_proxy.hpp"
#include <memory>

using namespace std;

/**
 * @brief Constructs an ArmProxy object.
 *
 * Initializes the ArmProxy instance with a specific arm type.
 *
 * @param type The type of robotic arm being managed.
 */
ArmProxy::ArmProxy(enum ArmType_e type)
 : type(type)
{
}

/**
 * @brief Initializes the ArmProxy.
 *
 * This method associates the proxy with a ROS2 node, allowing it to interact with
 * the arm control system.
 *
 * @param node Shared pointer to the ROS2 node.
 * @return int Returns 0 on successful initialization.
 */
int ArmProxy::init(shared_ptr<rclcpp::Node> node){
  this->node = node;
  //TODO: make service connections
  return 0;
}
