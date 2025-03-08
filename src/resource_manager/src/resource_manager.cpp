#include <memory>
#include <thread>
#include <iostream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "resource_manager/srv/request_resource.hpp"
#include "resource_database.h"
#include "resource_database_proxy.h"
#include "resource_request_respond.h"

using namespace std;

int main(int argc, char ** argv)
{
  // initializing ros communication
  rclcpp::init(argc, argv);

  shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("resource_manager");


  ResourceDataBaseProxy_t* safeDatabase = initResourceDatabaseProxy();

  // Create the service using a lambda function as the callback
  auto service = node->create_service<resource_manager::srv::RequestResource>(
    "request_resource",
    [safeDatabase](const std::shared_ptr<resource_manager::srv::RequestResource::Request> request,
       std::shared_ptr<resource_manager::srv::RequestResource::Response> response) -> void
    {
      // Log the received request
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Received request: a=%d, b=%d, c=%d", request->a, request->b, request->c);

      // change to listen on ros topic
      int requesterId = request->a;
      int reqResource = request->b;
      enum ResourceRequestType_e reqType = static_cast<ResourceRequestType_e>(request->c);
      cout << reqType << endl;

      // Process the request
      response->resp = respondRequest(safeDatabase, reqResource,requesterId,  reqType);

      // Log the response
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Sending response: resp=%d", response->resp);
    }
  );

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Resource Manager is ready");

  // Spin the executor; callbacks will be handled by multiple threads concurrently.
  executor.spin();

  // shutdown ross communication
  rclcpp::shutdown();
  return 0;
}
