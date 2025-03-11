#ifndef ARM_PROXY_H_
#define ARM_PROXY_H_

#include "arm.h"

class ArmProxy{
  public:
    ArmProxy(enum ArmType_e type);

  private:
    enum ArmType_e type;
};


#endif // ARM_PROXY_H_
