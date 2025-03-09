#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include "rclcpp/rclcpp.hpp"
#include <list>
#include "coordinator/turtle_proxy.h"

class Coordinator{
  public:
    int sendSetupMessages(shared_ptr<rclcpp::Node> node);

    int createTurtleServices(shared_ptr<rclcpp::Node> node);

    int createArmServices(shared_ptr<rclcpp::Node> node);

  private:
    list<TurtleProxy> registeredTurtles;
}




#endif // COORDINATOR_H_
