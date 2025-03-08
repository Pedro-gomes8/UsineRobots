#ifndef RESOURCE_REQUEST_RESPOND_H_
#define RESOURCE_REQUEST_RESPOND_H_

#include "resource_database_proxy.h"

enum ResourceRequestType_e {
  LOCK_REQUEST,
  RELEASE_REQUEST
};

typedef struct RespondRequestData_t RespondRequestData_t;

int respondRequest(ResourceDataBaseProxy_t* safeDatabase,
                    int resource, int requesterId, enum ResourceRequestType_e reqType);


#endif // RESOURCE_REQUEST_RESPOND_H_
