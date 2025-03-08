#ifndef RESOURCE_REQUEST_RESPOND_H_
#define RESOURCE_REQUEST_RESPOND_H_

#include "resource_database_proxy.h"

enum ResourceRequestType_e {
  LOCK_REQUEST,
  RELEASE_REQUEST
};

typedef struct ResopondRequestData_t ResopondRequestData_t;

void* respondRequest(void* data);

ResopondRequestData_t* bundleData(ResourceDataBaseProxy_t* safeDatabase,
                                  int requesterId,int resource,
                                  enum ResourceRequestType_e reqType);

#endif // RESOURCE_REQUEST_RESPOND_H_
