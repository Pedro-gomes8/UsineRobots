#include "resource_manager_proxy.hpp"

using namespace std;


int ResourceManagerProxy::init(shared_ptr<rclcpp::Node> node)
{
  this->node = node;
  this->client = node->create_client<RequestResource>("request_resource");
  return 0;
}

int ResourceManagerProxy::lockResource(enum ResourcesNames_e resource){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "attempting to lock resource %d", resource);

  shared_ptr<RequestResource::Request> req = make_shared<RequestResource::Request>();
  req->requester = 0; // id of the coordinator
  req->resource = resource;
  req->type = 0; // id to lock resource

  return this->syncSendRequest(req);
}

int ResourceManagerProxy::releaseResource(enum ResourcesNames_e resource){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "attempting to release resource %d", resource);
  shared_ptr<RequestResource::Request> req = make_shared<RequestResource::Request>();
  req->requester = 0; // id of the coordinator
  req->resource = resource;
  req->type = 1; // id to release resource

  return 0;
}

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
