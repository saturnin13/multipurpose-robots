
//ultrasonic sensor
const int trigPin = 52;
const int echoPin = 53;
const int trigPin2 = 50;
const int echoPin2 = 51;
long duration1;
int distance1;
long duration2;
int distance2;

//not reliable for distances lower than 3cm
void setup()
{
      //ultra sonic
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin, INPUT); // Sets the echoPin as an Input
      pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
      
      Serial.begin(9600);
}
 
void loop()
{
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state_machine for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration1 = pulseIn(echoPin, HIGH);

      digitalWrite(trigPin2, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin2, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin2, LOW);
      duration2 = pulseIn(echoPin2, HIGH);
      // Calculating the distance
      distance1= duration1*0.034/2;
      distance2= duration2*0.034/2;
      Serial.print(distance1);
      Serial.print(" and ");
      Serial.print(distance2);
      Serial.println(" cm");


      delay(100);
}
