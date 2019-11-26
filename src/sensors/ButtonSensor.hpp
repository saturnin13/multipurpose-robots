#ifndef BUTTONSENSOR_H
#define BUTTONSENSOR_H

#include "Sensor.hpp"

class ButtonSensor : public Sensor {

  public:
    ButtonSensor(int pin);

    void update();
    void reset();

    bool pressed = false;

  private:
    const int pin;
    unsigned long lastDebounceTime = 0;
    int previousButtonState = -1;

};

#endif