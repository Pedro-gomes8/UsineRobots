/**
 * @file resource_request_respond.h
 * @brief Handles resource request responses.
 *
 * This module provides functionality for responding to resource requests,
 * such as locking and releasing resources, through a resource database proxy.
 */
#ifndef RESOURCE_REQUEST_RESPOND_H_
#define RESOURCE_REQUEST_RESPOND_H_

#include "resource_database_proxy.hpp"

//TODO: move this to interface
/**
 * @brief Enum representing different types of resource requests.
 */
enum ResourceRequestType_e {
  LOCK_REQUEST,
  RELEASE_REQUEST
};

/**
 * @brief Processes a resource request.
 *
 * Handles a request to either lock or release a resource, ensuring safe access
 * via the resource database proxy.
 *
 * @param[in] safeDatabase Pointer to the resource database proxy instance.
 * @param[in] resource ID of the resource to be processed.
 * @param[in] requesterId ID of the requester.
 * @param[in] reqType Type of request (LOCK_REQUEST or RELEASE_REQUEST).
 * @return 0 on success, or a negative value on failure.
 * @note calls to this function MAY be blocking
 */
int respondRequest(ResourceDataBaseProxy_t* safeDatabase,
                    int resource, int requesterId, enum ResourceRequestType_e reqType);


#endif // RESOURCE_REQUEST_RESPOND_H_
