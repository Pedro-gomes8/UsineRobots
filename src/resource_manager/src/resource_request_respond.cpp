#include <iostream>
#include <pthread.h>
#include <stdlib.h>
#include "resource_request_respond.h"
#include "resource_database_proxy.h"

using namespace std;

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

/*
struct RespondRequestData_t{
  ResourceDataBaseProxy_t* safeDatabase;
  int requesterId;
  int resource;
  enum ResourceRequestType_e reqType;
  pthread_t tid;
};

RespondRequestData_t* bundleRespondRequestData(ResourceDataBaseProxy_t* safeDatabase,
                                  int requesterId,int resource,
                                  enum ResourceRequestType_e reqType);

void* respondRequestThread(void* data);

RespondRequestData_t* bundleRespondRequestData(ResourceDataBaseProxy_t* safeDatabase,
                                  int requesterId,int resource,
                                  enum ResourceRequestType_e reqType){

 RespondRequestData_t* data = (RespondRequestData_t*) malloc(sizeof(RespondRequestData_t));

 data->safeDatabase = safeDatabase;
 data->requesterId = requesterId;
 data->resource = resource;
 data->reqType = reqType;

 return data;
}
void* respondRequestThread(void* data){
  RespondRequestData_t* typedData = ( RespondRequestData_t* ) data;

  (void)pthread_detach(typedData->tid);
  ResourceDataBaseProxy_t* safeDatabase = typedData->safeDatabase;
  int requesterId = typedData->requesterId;
  int resource = typedData->resource;
  enum ResourceRequestType_e reqType = typedData->reqType;

  // send message back

  free(data);

  pthread_exit(NULL);
}

 */
