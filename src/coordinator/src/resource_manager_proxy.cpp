/**
 * @file resource_manager_proxy.cpp
 * @brief Implementation of the ResourceManagerProxy class.
 *
 * This file provides the method definitions for the ResourceManagerProxy class,
 * which facilitates resource management through a ROS2 service interface.
 */
#include "resource_manager_proxy.hpp"

using namespace std;

/**
 * @brief Initializes the ResourceManagerProxy.
 *
 * This function initializes the ResourceManagerProxy by setting up the ROS2 service client.
 *
 * @param node Shared pointer to the ROS2 node.
 * @return int Returns 0 on successful initialization.
 */
int ResourceManagerProxy::init(shared_ptr<rclcpp::Node> node)
{
  this->node = node;
  this->client = node->create_client<RequestResource>("request_resource");
  return 0;
}

/**
 * @brief Locks a specified resource.
 *
 * Sends a request to lock the given resource by calling the ROS2 service.
 *
 * @param resource The resource to be locked.
 * @return int Returns the response code from the service.
 */
int ResourceManagerProxy::lockResource(enum ResourcesNames_e resource){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "attempting to lock resource %d", resource);

  shared_ptr<RequestResource::Request> req = make_shared<RequestResource::Request>();
  req->requester = 0; // id of the coordinator
  req->resource = resource;
  req->type = 0; // id to lock resource

  return this->syncSendRequest(req);
}

/**
 * @brief Releases a specified resource.
 *
 * Sends a request to release the given resource by calling the ROS2 service.
 *
 * @param resource The resource to be released.
 * @return int Always returns 0.
 */
int ResourceManagerProxy::releaseResource(enum ResourcesNames_e resource){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "attempting to release resource %d", resource);
  shared_ptr<RequestResource::Request> req = make_shared<RequestResource::Request>();
  req->requester = 0; // id of the coordinator
  req->resource = resource;
  req->type = 1; // id to release resource

  return 0;
}

/**
 * @brief Sends a synchronous request to the resource management service.
 *
 * This method waits for the service to become available and then sends the request
 * synchronously. If the service is unavailable, it retries until available.
 *
 * @param req Shared pointer to the service request message.
 * @return int Returns the response code from the service.
 */
int ResourceManagerProxy::syncSendRequest(shared_ptr<RequestResource::Request> req){
  // connect to service
  while (!this->client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // send message
  auto result = client->async_send_request(req);
  // Wait for the result.
  return result.get()->resp;

}
