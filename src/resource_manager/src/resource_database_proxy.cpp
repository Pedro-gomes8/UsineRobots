#include <iostream>
#include <err.h>
#include <semaphore.h>
#include <stdlib.h>

#include "resource_database.hpp"
#include "resource_database_proxy.hpp"

/**
 * @brief simple wrapper around the array implementation of the resource
 *        database that synchronizes access via a semaphore
 *
 * @param database Database being protected
 * @param lock Semaphore used to protect the database
 */
struct ResourceDataBaseProxy_t{
  ResourceDataBase_t* database;
  sem_t lock;
};

/**
 * @brief Initializes the resource database proxy.
 *
 * Allocates and initializes a new instance of the resource database proxy.
 *
 * @return Pointer to the initialized ResourceDataBaseProxy_t instance, or NULL on failure.
 */
ResourceDataBaseProxy_t* initResourceDatabaseProxy(void) {
  ResourceDataBaseProxy_t* db_proxy =
      (ResourceDataBaseProxy_t*)malloc(sizeof(ResourceDataBaseProxy_t));

  if (db_proxy == NULL) {
    err(EXIT_FAILURE, "malloc");
  }

  db_proxy->database = initResourceDataBase();

  if (sem_init(&db_proxy->lock, 0, 1) != 0) {
    err(EXIT_FAILURE, "sem_init");
  }

  return db_proxy;
}

/**
 * @brief Shuts down and deallocates the resource database proxy.
 *
 * Releases any resources associated with the given resource database proxy.
 *
 * @param[in] dbProxy Pointer to the resource database proxy instance.
 * @return 0 on success, or a negative value on failure.
 */
int endResourceDataBaseProxy(ResourceDataBaseProxy_t* dbProxy){
  int resDb = endResourceDataBase(dbProxy->database);
  if(resDb != 0){
    return resDb;
  }
  free(dbProxy);

  return 0;
}

/**
 * @brief Attempts to lock a specific resource through the proxy.
 *
 * Tries to acquire a lock on the resource identified by ressourceId.
 *
 * @param[in] db_proxy Pointer to the resource database proxy instance.
 * @param[in] ressourceId ID of the resource to lock.
 * @param[in] requesterId ID of who is requesting the resource
 * @return 0 if the lock was acquired successfully, or a negative value on failure.
 */
int attemptLockResourceProxy(ResourceDataBaseProxy_t* db_proxy,
                              int ressourceId, int requesterId) {
  sem_wait(&db_proxy->lock);
  int res = 0;
  if (attemptLockResource(db_proxy->database, ressourceId, requesterId) != 0) {
    res = -1;
  }

  sem_post(&db_proxy->lock);
  return res;
}

/**
 * @brief Releases a locked resource through the proxy.
 *
 * Unlocks the resource identified by ressourceId.
 *
 * @param[in] db_proxy Pointer to the resource database proxy instance.
 * @param[in] ressourceId ID of the resource to release.
 * @param[in] requesterId ID of who is requesting the resource
 * @return 0 on success, or a negative value on failure.
 */
int releaseResourceProxy(ResourceDataBaseProxy_t* db_proxy, int ressourceId, int requesterId) {
  sem_wait(&db_proxy->lock);

  int res = releaseResource(db_proxy->database, ressourceId, requesterId);

  sem_post(&db_proxy->lock);
  return res;
}

/**
 * @brief Waits for a resource to become available through the proxy.
 *
 * Blocks until the resource identified by ressourceId is available.
 *
 * @param[in] database Pointer to the resource database proxy instance.
 * @param[in] ressourceId ID of the resource to wait for.
 * @return 0 when the resource becomes available, or a negative value on failure.
 */
int waitResourceProxy(ResourceDataBaseProxy_t* database, int ressourceId){
  return waitResource(database->database, ressourceId);
}

/**
 * @brief Register a resource that HAS NOT YET been initialized
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to lock.
 * @param[in] ammount The ammount of times this resource can be unlocked without consequences
 * @return 0 if the lock was acquired successfully, or a negative value on failure.
 * @note does not manage access to the database
 */
int registerResourceProxy(ResourceDataBaseProxy_t* database, int ressourceId, int ammount){
    registerResource(database->database, ressourceId, ammount);
    return 0;
}
