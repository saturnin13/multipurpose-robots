#include "LED.hpp"
#include <Arduino.h>

LED::LED(int redPin, int greenPin, int bluePin)
    : redPin(redPin), greenPin(greenPin), bluePin(bluePin) {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
}

void LED::configure(LEDState state) {
    this->state = state;
    this->cycleState = false;
    this->lastCycleTime = 0;

    switch (state) {
    case SOLID_RED:
        digitalWrite(redPin, HIGH);
        digitalWrite(bluePin, LOW);
        digitalWrite(greenPin, LOW);
        break;

    case STROBE_GREEN:
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, LOW);
        break;

    case FLASH_BLUE:
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, LOW);
        break;
    }

}

void LED::enact() {

    unsigned long now = millis();
    unsigned long elapsed = now - lastCycleTime;

    switch (state) {
    case SOLID_RED:
        break;

    case FLASH_BLUE:
        if (elapsed > FLASH_FREQUENCY) {
            if (this->cycleState) {
                // Light is on
                digitalWrite(bluePin, LOW);
            } else {
                // Light is off
                digitalWrite(bluePin, HIGH);
            }

            this->lastCycleTime = now;
            this->cycleState = !this->cycleState;
        }

        break;

    case STROBE_GREEN:

        if (this->cycleState) {
            // Light is on high brightness
            if (elapsed > STROBE_HIGH_DURATION) {
                analogWrite(greenPin, LED_DIM);
                this->lastCycleTime = now;
                this->cycleState = !this->cycleState;
            }
        } else {
            // Light is on low brightness
            if (elapsed > STROBE_LOW_DURATION) {
                analogWrite(greenPin, LED_BRIGHT);
                this->lastCycleTime = now;
                this->cycleState = !this->cycleState;
            }
        }

        break;
    }
}