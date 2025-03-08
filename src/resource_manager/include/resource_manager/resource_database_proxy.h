/**
 * @file resource_database_proxy.h
 * @brief Proxy interface for the Resource Database.
 *
 * This module provides a proxy layer for managing and synchronizing access
 * to resources through a resource database. It includes functions for
 * initialization, locking, releasing, and waiting on resources.
 */
#ifndef RESOURCE_DATABASE_PROXY_H_
#define RESOURCE_DATABASE_PROXY_H_

#include "resource_database.h"

/**
 * @brief Opaque structure representing a resource database proxy.
 */
typedef struct ResourceDataBaseProxy_t ResourceDataBaseProxy_t;

/**
 * @brief Initializes the resource database proxy.
 *
 * Allocates and initializes a new instance of the resource database proxy.
 *
 * @return Pointer to the initialized ResourceDataBaseProxy_t instance, or NULL on failure.
 */
ResourceDataBaseProxy_t* initResourceDatabaseProxy(void);

/**
 * @brief Shuts down and deallocates the resource database proxy.
 *
 * Releases any resources associated with the given resource database proxy.
 *
 * @param[in] dbProxy Pointer to the resource database proxy instance.
 * @return 0 on success, or a negative value on failure.
 */
int endResourceDataBaseProxy(ResourceDataBaseProxy_t* dbProxy);

/**
 * @brief Attempts to lock a specific resource through the proxy.
 *
 * Tries to acquire a lock on the resource identified by ressourceId.
 *
 * @param[in] db_proxy Pointer to the resource database proxy instance.
 * @param[in] ressourceId ID of the resource to lock.
 * @return 0 if the lock was acquired successfully, or a negative value on failure.
 */
int attemptLockResourceProxy(ResourceDataBaseProxy_t* db_proxy,
                              int ressourceId, int requesterId);

/**
 * @brief Releases a locked resource through the proxy.
 *
 * Unlocks the resource identified by ressourceId.
 *
 * @param[in] db_proxy Pointer to the resource database proxy instance.
 * @param[in] ressourceId ID of the resource to release.
 * @return 0 on success, or a negative value on failure.
 */
int releaseResourceProxy(ResourceDataBaseProxy_t* db_proxy, int ressourceId, int requesterId);

/**
 * @brief Waits for a resource to become available through the proxy.
 *
 * Blocks until the resource identified by ressourceId is available.
 *
 * @param[in] database Pointer to the resource database proxy instance.
 * @param[in] ressourceId ID of the resource to wait for.
 * @return 0 when the resource becomes available, or a negative value on failure.
 */
int waitResourceProxy(ResourceDataBaseProxy_t* database, int ressourceId);

#endif  // RESSOURCE_DATABASE_PROXY_H_
