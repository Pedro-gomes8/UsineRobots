/**
 * @file resource_database.h
 * @brief Resource Database management interface.
 *
 * This module provides functions to initialize, and manage access to resources
 * in a resource database. It includes functionality for locking, releasing, and
 * waiting on resources.
 */

#ifndef RESSOURCE_DATABASE_H_
#define RESSOURCE_DATABASE_H_

/**
 * @brief Opaque structure representing a resource database.
 *
 * TODO(andre): Convert this into an interface.
 */
typedef struct ResourceDataBase_t ResourceDataBase_t;

/**
 * @brief Initializes the resource database.
 *
 * Allocates and initializes a new instance of the resource database.
 *
 * @return Pointer to the initialized ResourceDataBase_t instance, or NULL on failure.
 */
ResourceDataBase_t* initResourceDataBase(void);

/**
 * @brief Shuts down and deallocates the resource database.
 *
 * Releases any resources associated with the given resource database.
 *
 * @param[in] database Pointer to the resource database instance.
 * @return 0 on success, or a negative value on failure.
 */
int endResourceDataBase(ResourceDataBase_t* database);

/**
 * @brief Attempts to lock a specific resource.
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to lock.
 * @return 0 if the lock was acquired successfully, or a negative value on failure.
 */
int attemptLockResource(ResourceDataBase_t* database, int ressourceId);

/**
 * @brief Releases a locked resource.
 *
 * Unlocks the resource identified by ressourceId.
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to release.
 * @return 0 on success, or a negative value on failure.
 */
int releaseResource(ResourceDataBase_t* database, int ressourceId);

/**
 * @brief Waits for a resource to become available.
 *
 * Blocks until the resource identified by ressourceId is available.
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to wait for.
 * @return 0 when the resource becomes available, or a negative value on failure.
 */
int waitResource(ResourceDataBase_t* database, int ressourceId);

#endif  // RESSOURCE_DATABASE_H_
