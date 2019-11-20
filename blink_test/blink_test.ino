const int ledBlue = 52;
const int ledGreen = 13; //has to be PWM
const int ledRed = 53;
const int buttonPin = 51; 

enum State {
  DISSARMED,
  COMPLETED,
  ARMED,
};

enum GreenLED {
  GREENOFF,
  GREENLOW,
  GREENON,
};

int flashingDelay = 500;
int strobingOn = 100;
const int strobingLow = 1200;

unsigned long lastSwitchArmed;
unsigned long lastSwitchCompleted;
int state;
int greenLED;
int buttonState;

//debounce
int lastButtonState;   // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers
bool buttonChanged = false;

int ledOn = 255;
int ledLow = 100;
int ledOff = 0;



void setup() {
// put your setup code here, to run once:
pinMode(ledBlue, OUTPUT);
pinMode(ledGreen, OUTPUT);
pinMode(ledRed, OUTPUT);
pinMode(buttonPin, INPUT_PULLUP);
lastSwitchCompleted = millis();
lastSwitchArmed = millis();
state = DISSARMED;
lastButtonState = digitalRead(buttonPin);
greenLED = GREENOFF; 

Serial.begin(9600);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (buttonState != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    Serial.println("Timer started");
  }

  if ((millis() - lastDebounceTime) > debounceDelay && lastDebounceTime != 0) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state machine:
    Serial.println("Button pressed!");
    lastDebounceTime = 0;
    // if the button state machine has changed:
    buttonChanged  = 1;
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = buttonState;

  
  if(buttonChanged) {
    buttonChanged = 0;
     Serial.println("Changing state machine!");

    if (state == DISSARMED) {
      state = ARMED;
    } else if (state == ARMED) {
      state = COMPLETED;
    } else if (state == COMPLETED) {
      state = DISSARMED;
    }
  }
  
  switch(state){
    case DISSARMED:
      digitalWrite(ledBlue, LOW);
      analogWrite(ledGreen, ledOff);
      digitalWrite(ledRed, HIGH);
      break;
    case COMPLETED: 
      digitalWrite(ledRed, LOW);
      analogWrite(ledGreen, ledOff);
      if((millis() - lastSwitchCompleted) > flashingDelay) {
        lastSwitchCompleted = millis();
        digitalWrite(ledBlue, !digitalRead(ledBlue));
      }      
      break;
    case ARMED:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledBlue, LOW);
      if((millis() - lastSwitchArmed) > strobingLow && greenLED == GREENLOW || greenLED == GREENOFF) {
        greenLED = GREENON;
        lastSwitchArmed = millis();
        analogWrite(ledGreen, ledOn);
      } else if ((millis() - lastSwitchArmed) > strobingOn && greenLED == GREENON) {
        greenLED = GREENLOW;
        lastSwitchArmed = millis();
        analogWrite(ledGreen, ledLow);
      }
      break;
    default:
      digitalWrite(ledBlue, LOW);
      analogWrite(ledGreen, 0);
      digitalWrite(ledRed, LOW);
  break;
  }

  delay(100);
}
