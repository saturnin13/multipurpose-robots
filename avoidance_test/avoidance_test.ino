const int tracingPin = 0;

void setup()
{
pinMode(tracingPin, INPUT);
        Serial.begin(9600);

}
void loop()
{
int val = digitalRead(tracingPin);
if(val == 0)
{ 
Serial.println(val);}
else
{
Serial.println(val);
}
delay(100);
}
