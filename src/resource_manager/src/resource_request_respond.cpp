#include <iostream>
#include <pthread.h>
#include <stdlib.h>
#include "resource_request_respond.h"
#include "resource_database_proxy.h"

using namespace std;

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
                    int resource, int requesterId, enum ResourceRequestType_e reqType){

  int res = 0;
  int waitRes = 0;
  fflush(stdout);
  switch(reqType){
    case LOCK_REQUEST:
      cout<< "test" << endl;
      waitRes = waitResourceProxy(safeDatabase, resource);
      if(waitRes != 0){
        res = waitRes;
        break;
      }
      res = attemptLockResourceProxy(safeDatabase, resource, requesterId);
      break;

    case RELEASE_REQUEST:
      res = releaseResourceProxy(safeDatabase, resource, requesterId);
      break;

    default:
      // in case new resource request types get added to enum but not here
      break;
  }

  return res;
}
