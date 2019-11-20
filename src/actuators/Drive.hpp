#ifndef DRIVE_H
#define DRIVE_H

#include "Actuator.hpp"

class Drive : public Actuator {
  public:
    LED();
    void enact();
};

#endif
