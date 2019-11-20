
//ultrasonic sensor
const int trigPin = 12;
const int echoPin = 11;
long duration;
int distance;

//not reliable for distances lower than 3cm
void setup()
{
      //ultra sonic
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin, INPUT); // Sets the echoPin as an Input
      
      Serial.begin(9600);
}
 
void loop()
{
     digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance= duration*0.034/2;
      Serial.print(distance);
      Serial.println(" cm");


      delay(100);
}
