#include <memory>
#include <thread>
#include <iostream>
#include <cstdio>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "coordinator.hpp"

using namespace std;

int main(int argc, char ** argv)
{
    // initializing ros communication
    rclcpp::init(argc, argv);

    // Create a reentrant callback group to allow concurrent execution.

    shared_ptr<Coordinator> node = std::make_shared<Coordinator>();

    //coord.createTurtleServices(node);

    //coord.createArmServices(node);

    // allows for callbacks to be executed in parallel
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    //coord.sendSetupMessages(node);

    // send permission message to arm robot

    rclcpp::shutdown();
    return 0;
}
