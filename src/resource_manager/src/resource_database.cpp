#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>

#include "resource_database.h"

#define MAX_RESOURCE_ID 50

/**
 * @brief simple array implementation of the resource database
 *
 * @param registered boolean array to indicate whether
 * @param availability int array indicating the ammount available of each resource
 * @param interest semaphores blocking the threads interested in a given resource
 *
 * TODO(andre): Convert this into an interface.
 */
struct ResourceDataBase_t{
  bool registered[MAX_RESOURCE_ID];
  int availability[MAX_RESOURCE_ID];
  sem_t interest[MAX_RESOURCE_ID];
};

/**
 * @brief Register a resource that HAS NOT YET been initialized
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to lock.
 * @return 0 if the lock was acquired successfully, or a negative value on failure.
 * @note does not manage access to the database
 */
int registerResource(ResourceDataBase_t* database, int ressourceId);

/**
 * @brief Initializes the resource database.
 *
 * Allocates and initializes a new instance of the resource database.
 *
 * @return Pointer to the initialized ResourceDataBase_t instance, or NULL on failure.
 */
ResourceDataBase_t *initResourceDataBase() {
  ResourceDataBase_t *database =
      (ResourceDataBase_t *)malloc(sizeof(ResourceDataBase_t));

  if (database == NULL) {
    err(EXIT_FAILURE, "malloc");
  }

  (void)memset(database->registered, 0, MAX_RESOURCE_ID*sizeof(bool));
  (void)memset(database->availability, 0, MAX_RESOURCE_ID*sizeof(int));

  for(int i=0; i<MAX_RESOURCE_ID; i++){
    sem_init(&(database->interest[i]),0,1);
  }

  return database;
}

/**
 * @brief Shuts down and deallocates the resource database.
 *
 * Releases any resources associated with the given resource database.
 *
 * @param[in] database Pointer to the resource database instance.
 * @return 0 on success, or a negative value on failure.
 */
int endResourceDataBase(ResourceDataBase_t* database){
  // destroy semaphores
  for(int i=0; i<MAX_RESOURCE_ID; i++){
    sem_destroy(&(database->interest[i]));
  }

  free(database);
  return 0;
}

/**
 * @brief Attempts to lock a specific resource.
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to lock.
 * @return 0 if the lock was acquired successfully, or a negative value on failure.
 * @note does not manage access to the database
 */
int attemptLockResource(ResourceDataBase_t *database, int ressourceId) {
  // ressource Id out of bounds
  if (ressourceId < 0 || ressourceId >= MAX_RESOURCE_ID) {
    return -1;
  }

  // if ressource is not registered, register it
  if (!database->registered[ressourceId]) {
    (void)registerResource(database, ressourceId);
  }

  if(database->availability[ressourceId] < 1){
    return -1;
  }

  database->availability[ressourceId] -= 1;
  return 0;
}

/**
 * @brief Releases a locked resource.
 *
 * Unlocks the resource identified by ressourceId.
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to release.
 * @return 0 on success, or a negative value on failure.
 */
int releaseResource(ResourceDataBase_t *database, int ressourceId) {
  // if ressource is not registered, return error
  if (!database->registered[ressourceId]) {
    return -1;
  }
  database->availability[ressourceId] +=1;

  (void)sem_post(&(database->interest[ressourceId]));
  return 0;
}

/**
 * @brief Waits for a resource to become available.
 *
 * Blocks until the resource identified by ressourceId is available.
 *
 * @param[in] database Pointer to the resource database instance.
 * @param[in] ressourceId ID of the resource to wait for.
 * @return 0 when the resource becomes available, or a negative value on failure.
 */
int waitResource(ResourceDataBase_t* database, int ressourceId){
  return sem_wait(&(database->interest[ressourceId]));;
}

int registerResource(ResourceDataBase_t* database, int ressourceId){
    database->registered[ressourceId] = true;
    database->availability[ressourceId] = 1;

    return 0;
}
