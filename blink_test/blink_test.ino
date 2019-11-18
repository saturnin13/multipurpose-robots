int ledBlue = 52;
int ledGreen = 13;
int ledRed = 53;

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


int ledOn = 255;
int ledLow = 100;
int ledOff = 0;

void setup() {
  // put your setup code here, to run once:
pinMode(ledBlue, OUTPUT);
pinMode(ledGreen, OUTPUT);
pinMode(ledRed, OUTPUT);

lastSwitchCompleted = millis();
lastSwitchArmed = millis();
state = DISSARMED;

greenLED = GREENOFF; 

Serial.begin(9600);
}

void loop() {
  
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
