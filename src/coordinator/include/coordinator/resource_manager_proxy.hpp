#ifndef RESOURCE_MANAGER_PROXY_H_
#define RESOURCE_MANAGER_PROXY_H_

#include "rclcpp/rclcpp.hpp"
#include "resource_manager_interface/srv/request_resource.hpp"
#include "resource_manager_interface.hpp"

using namespace std;

using RequestResource = resource_manager_interface::srv::RequestResource;

class ResourceManagerProxy{
  public:
    int init(shared_ptr<rclcpp::Node> node);

    int lockResource(enum ResourcesNames_e resource);

    int releaseResource(enum ResourcesNames_e resource);

    int syncSendRequest(shared_ptr<RequestResource::Request> req);

  private:
    rclcpp::Client<RequestResource>::SharedPtr client;
    shared_ptr<rclcpp::Node> node;
};

#endif // RESOURCE_MANAGER_PROXY_H_
