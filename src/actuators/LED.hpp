#ifndef LED_H
#define LED_H

#include "Actuator.hpp"

#define FLASH_RATE 500
#define STROBE_HIGH_DURATION 100
#define STROBE_LOW_DURATION 1200
#define LED_BRIGHT 255
#define LED_DIM 90

enum LEDState { SOLID_RED, STROBE_GREEN, FLASH_BLUE };

class LED : public Actuator {
  public:
    LED(int redPin, int greenPin, int bluePin);

    void configure(LEDState state);
    void enact();

  private:
    const int redPin;
    const int greenPin;
    const int bluePin;

    LEDState state;
    unsigned long lastCycleTime = 0;
    bool cycleState = false;
};

#endif