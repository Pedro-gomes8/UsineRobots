#ifndef TURTLE_PROXY_H_
#define TURTLE_PROXY_H_
#include "turtle.h"
#include <string>
using namespace std;

class TurtleProxy{
  public:
    TurtleProxy(TurtlePosition_e position, string contentColor, int cargoLimit);

    int changeTurtlePosition(TurtlePosition_e newPosition);

    bool isFull();

    bool isEmpty();

    bool cargoHasColor(string referenceColor);

    int changeCargoAmmount(int ammount);

    int changeCargoType(string cargoColor);

  private:
    int cargoAmmount;
    TurtlePosition_e position;
    string contentColor;
    int cargoLimit;

};



#endif // TURTLE_PROXY_H_
