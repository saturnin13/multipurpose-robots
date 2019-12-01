int IN1 = 4;
int IN2 = 5;
int ENA1 = 8;

int IN3 = 6;
int IN4 = 7;
int ENA2 = 9;

void setup() {
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(ENA1, OUTPUT);

pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
pinMode(ENA2, OUTPUT);

Serial.begin(9600);

Serial.println("Setup done");
}

void loop() {

Serial.println("First Motor Test forward");
digitalWrite(IN1, LOW); //forward
digitalWrite(IN2, HIGH); //forward
analogWrite(ENA1, 40); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255.
digitalWrite(IN3, LOW); //forward
digitalWrite(IN4, HIGH); //forward
analogWrite(ENA2, 40); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255.
delay(30000);
/*Serial.println("First Motor Test forward");
digitalWrite(IN1, LOW); //forward
digitalWrite(IN2, HIGH); //forward
analogWrite(ENA1, 178); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255.
delay(3000);

Serial.println("First Motor Test backward");
digitalWrite(IN1, HIGH); //reverse
digitalWrite(IN2, LOW); //reverse
analogWrite(ENA1, 76); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255.
delay(3000);

Serial.println("Second Motor Test forward");
digitalWrite(IN3, LOW); //forward
digitalWrite(IN4, HIGH); //forward
analogWrite(ENA2, 178); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255.
delay(3000);

Serial.println("Second Motor Test backward");
digitalWrite(IN3, HIGH); //reverse
digitalWrite(IN4, LOW); //reverse
analogWrite(ENA2, 76); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255.
delay(3000);*/
}
