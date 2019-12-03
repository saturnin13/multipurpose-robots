#include "ButtonUpdateAgent.hpp"
#include "../../Constants.hpp"

ButtonUpdateAgent::ButtonUpdateAgent(State *state, ButtonSensor *button)
    : UpdateAgent(state), button(button) {
    this->armingTime = 0;
}
void ButtonUpdateAgent::update() {

    if (DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG) {
        Serial.print("\nButtonUpdateAgent: ");
    }

    // Start arming countdown if button is pressed.
    if (this->button->pressed && this->state->robotState == DISARMED && !this->state->emergencyStop) {
        if (DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG) {
            Serial.println("ARMED");
        }
        this->state->robotState = ARMED;
        this->armingTime = millis();
        return;
    }

    // Arm the robot.
    if (this->state->robotState == ARMED && !this->state->move &&
        (millis() - this->armingTime) > ARMING_DELAY) {

        if (DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG) {
            Serial.println("ENABLING MOVEMENT");
        }
        this->state->move = true;
    }

}