#include <thread>
#include <iostream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "resource_database.h"
#include "resource_database_proxy.h"
#include "resource_request_respond.h"

using namespace std;


typedef struct{
  ResourceDataBaseProxy_t* safeDatabase;
  int requesterId;
  int resource;
  enum ResourceRequestType_e reqType;
}ResopondRequestData_t;

void* respondRequest(void* data){
  ResourceDataBase_t* typedData = (ResourceDataBase_t*)data;

  pthread_detach();
  switch(reqType){
    case LOCK_REQUEST:
      (void)waitResourceProxy(safeDatabase, resource);
      attemptLockResourceProxy(safeDatabase, resource);
      break;

    case RELEASE_REQUEST:
      releaseResourceProxy(safeDatabase, resource);
      break;

    default:
      // in case new resource request types get added to enum but not here
      break;
  }
  // send message back
}

int main(int argc, char ** argv)
{
  // initializing ros communication
  rclcpp::init(argc, argv);

  // loop reading ros topics
  ResourceDataBase_t* safeDatabase = initResourceDataBase();
  while (1){
    // change to listen on ros topic
    int requesterId;
    int reqResource;
    enum ResourceRequestType_e reqType;
    //TODO: get reqType
    cin >> requesterId >> reqResource;
    // format input into just an int

    // create a thread to treat it and ignore output
    jthread responder{respondRequest, safeDatabase, requesterId, reqResource, reqType};
  }

  // shutdown ross communication
  rclcpp::shutdown();
  return 0;
}
