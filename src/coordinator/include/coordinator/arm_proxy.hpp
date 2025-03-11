/**
 * @file arm_proxy.hpp
 * @brief Defines the ArmProxy class.
 *
 * This file contains the declaration of the ArmProxy class, which serves as an interface
 * for managing robotic arms in a ROS2 environment.
 */
#ifndef ARM_PROXY_H_
#define ARM_PROXY_H_

#include "rclcpp/rclcpp.hpp"
#include "arm_interface.hpp"
#include <memory>

using namespace std;

/**
 * @class ArmProxy
 * @brief Proxy class for interacting with the robotic arm.
 *
 * The ArmProxy class provides an abstraction for initializing and managing a robotic arm
 * based on its type.
 */
class ArmProxy{
  public:
    /**
     * @brief Constructs an ArmProxy object.
     *
     * Initializes the ArmProxy instance with a specific arm type.
     *
     * @param type The type of robotic arm being managed.
     */
    ArmProxy(enum ArmType_e type);

    /**
     * @brief Initializes the ArmProxy.
     *
     * This method associates the proxy with a ROS2 node, allowing it to interact with
     * the arm control system.
     *
     * @param node Shared pointer to the ROS2 node.
     * @return int Returns 0 on successful initialization.
     */
    int init(shared_ptr<rclcpp::Node> node);

  private:
    shared_ptr<rclcpp::Node> node;
    enum ArmType_e type;
};


#endif // ARM_PROXY_H_
