#ifndef LED_H
#define LED_H

#include "Actuator.hpp"

class LED : public Actuator {
  public:
    LED();
    void enact();
};

#endif