#include <stdio.h>

int LinePin1 = 0;
int LinePin2 = 1;
int LinePin3 = 2;
int LinePin4 = 3;
int LinePin5 = 4;

void setup() {
  // put your setup code here, to run once:
        Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int l1 = analogRead(LinePin1);
  int l2 = analogRead(LinePin2);
  int l3 = analogRead(LinePin3);
  int l4 = analogRead(LinePin4);
  int l5 = analogRead(LinePin5);
  //Serial.print("Value: ");
  Serial.print(l1);Serial.print(" , ");
  Serial.print(l2);Serial.print(" , ");
  Serial.print(l3);Serial.print(" , ");
  Serial.print(l4);Serial.print(" , ");
  Serial.println(l5);
  delay(100);

  
}
