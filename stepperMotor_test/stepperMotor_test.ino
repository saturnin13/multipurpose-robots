//motor
const int stepperPin1 = 8;
const int stepperPin2 = 9;
const int stepperPin3 = 10;
const int stepperPin4 = 11;
int stepNumber = 0;
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 0

void setup()
{
      //motor
      pinMode(stepperPin1, OUTPUT);
      pinMode(stepperPin2, OUTPUT);
      pinMode(stepperPin3, OUTPUT);
      pinMode(stepperPin4, OUTPUT);
      
      Serial.begin(115200);
}
 
void loop()
{
      //start motor
      OneStep(COUNTERCLOCKWISE);
      delay(2);
}

void OneStep(int direction){ //step motor one step
  if (direction == 1){
      switch(stepNumber){
            case 0:
            digitalWrite(stepperPin1, LOW);
            digitalWrite(stepperPin2, LOW);
            digitalWrite(stepperPin3, LOW);
            digitalWrite(stepperPin4, HIGH);
            break;
            case 1:
            digitalWrite(stepperPin1, LOW);
            digitalWrite(stepperPin2, LOW);
            digitalWrite(stepperPin3, HIGH);
            digitalWrite(stepperPin4, LOW);
            break;
            case 2:
            digitalWrite(stepperPin1, LOW);
            digitalWrite(stepperPin2, HIGH);
            digitalWrite(stepperPin3, LOW);
            digitalWrite(stepperPin4, LOW);
            break;
            case 3:
            digitalWrite(stepperPin1, HIGH);
            digitalWrite(stepperPin2, LOW);
            digitalWrite(stepperPin3, LOW);
            digitalWrite(stepperPin4, LOW);
        }
    }else if (direction ==0)
    {
      switch(stepNumber){
            case 3:
            digitalWrite(stepperPin1, LOW);
            digitalWrite(stepperPin2, LOW);
            digitalWrite(stepperPin3, LOW);
            digitalWrite(stepperPin4, HIGH);
            break;
            case 2:
            digitalWrite(stepperPin1, LOW);
            digitalWrite(stepperPin2, LOW);
            digitalWrite(stepperPin3, HIGH);
            digitalWrite(stepperPin4, LOW);
            break;
            case 1:
            digitalWrite(stepperPin1, LOW);
            digitalWrite(stepperPin2, HIGH);
            digitalWrite(stepperPin3, LOW);
            digitalWrite(stepperPin4, LOW);
            break;
            case 0:
            digitalWrite(stepperPin1, HIGH);
            digitalWrite(stepperPin2, LOW);
            digitalWrite(stepperPin3, LOW);
            digitalWrite(stepperPin4, LOW);
      }
    }else{
      //print error
    }
      
      stepNumber++;
      if(stepNumber > 3){
            stepNumber = 0;
      }
}
