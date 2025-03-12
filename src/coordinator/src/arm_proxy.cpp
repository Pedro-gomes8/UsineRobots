/**
 * @file arm_proxy.cpp
 * @brief Implementation of the ArmProxy class.
 *
 * This file provides the method definitions for the ArmProxy class,
 * which serves as an interface for managing robotic arms in a ROS2 environment.
 */
#include "rclcpp/rclcpp.hpp"
#include "arm_proxy.hpp"
#include "turtle.h"
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
  this->client = node->create_client<TurtleBotArrived>("turtle_bot_arrived");
  return 0;
}

/*
 * utilizes a ros2 service
 */
int ArmProxy::notifyTurtleArrived(int turtleId, string cargoColor, float x, float y){

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "creating request object to notify arm");
  shared_ptr<TurtleBotArrived::Request> req = make_shared<TurtleBotArrived::Request>();
  req->id = turtleId;
  req->x_pos = x;
  req->y_pos = y;
  req->color = cargoColor;

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
