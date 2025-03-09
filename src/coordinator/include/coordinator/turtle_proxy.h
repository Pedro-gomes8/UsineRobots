#ifndef TURTLE_PROXY_H_
#define TURTLE_PROXY_H_
#include "turtle_coordinator_interface/include/turtle.h"
#include "arm_coordinator_interface/include/objects.h"

class TurtleProxy{
  public:
    int changeTurtlePosition(TurtlePosition_e newPosition);

    int changeTurtleCargo(ObjectColor_e cargoColor);

    TurtleProxy(TurtlePosition_e initialPosition);


  private:
    int turtleId;
    TurtlePosition_e position;
    ObjectColor_e carrying;
}



#endif // TURTLE_PROXY_H_
