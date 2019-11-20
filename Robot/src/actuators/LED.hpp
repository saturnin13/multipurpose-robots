#ifndef LED_H
#define LED_H

#include "Actuator.hpp"

#define FLASH_FREQUENCY 30
#define STROBE_HIGH_DURATION 100
#define STROBE_LOW_DURATION 1200
#define LED_BRIGHT 255
#define LED_DIM 80

enum LEDState { SOLID_RED, STROBE_GREEN, FLASH_BLUE };

class LED : public Actuator {
  public:
    LED(int redPin, int greenPin, int bluePin);

    void configure(LEDState state);
    void tick();

  private:
    const int redPin;
    const int greenPin;
    const int bluePin;

    LEDState state = SOLID_RED;
    unsigned long lastCycleTime = 0;
    bool cycleState = false;
};

#endif