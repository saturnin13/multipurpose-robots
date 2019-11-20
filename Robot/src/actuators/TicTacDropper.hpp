#ifndef TICTACDROPPER_H
#define TICTACDROPPER_H

#include "Actuator.hpp"

class TicTacDropper : public Actuator {
  public:
    TicTacDropper();
    void enact();
};

#endif