/**
 * @file resource_manager_proxy.hpp
 * @brief Defines the ResourceManagerProxy class.
 *
 * This file contains the declaration of the ResourceManagerProxy class,
 * which provides methods to request and release resources through a ROS2 service.
 */
#ifndef RESOURCE_MANAGER_PROXY_H_
#define RESOURCE_MANAGER_PROXY_H_

#include "rclcpp/rclcpp.hpp"
#include "resource_manager_interface/srv/request_resource.hpp"
#include "resource_manager_interface.hpp"

using namespace std;

using RequestResource = resource_manager_interface::srv::RequestResource;

/**
 * @class ResourceManagerProxy
 * @brief Proxy class for managing resource requests via ROS2 services.
 *
 * This class provides methods to lock and release resources by interacting with
 * the resource management service in a ROS2 environment.
 */
class ResourceManagerProxy{
  public:
    /**
     * @brief Initializes the ResourceManagerProxy.
     *
     * This method sets up the ROS2 service client for resource requests.
     *
     * @param node Shared pointer to the ROS2 node.
     * @return int Returns 0 on successful initialization.
     */
    int init(shared_ptr<rclcpp::Node> node);

    /**
     * @brief Requests to lock a specified resource.
     *
     * Sends a request to the resource management service to lock the given resource.
     *
     * @param resource The resource to be locked.
     * @return int The response code from the service (0 for success, negative for failure).
     */
    int lockResource(enum ResourcesNames_e resource);

    /**
     * @brief Requests to release a specified resource.
     *
     * Sends a request to the resource management service to release the given resource.
     *
     * @param resource The resource to be released.
     * @return int The response code from the service (0 for success, negative for failure).
     */
    int releaseResource(enum ResourcesNames_e resource);

    /**
     * @brief Sends a synchronous request to the resource management service.
     *
     * This method waits for the service to become available and sends a request
     * to lock or release a resource.
     *
     * @param req Shared pointer to the request message.
     * @return int The response code from the service (0 for success, negative for failure).
     */
    int syncSendRequest(shared_ptr<RequestResource::Request> req);

  private:
    rclcpp::Client<RequestResource>::SharedPtr client;
    shared_ptr<rclcpp::Node> node;
};

#endif // RESOURCE_MANAGER_PROXY_H_
