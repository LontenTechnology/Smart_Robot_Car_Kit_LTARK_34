#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int i=90;
void setup()
{
  Serial.begin(9600); 
  myservo.attach(8);  // modify each pin to adjust 
  myservo.write(i);
  delay(1000);
}

void loop() 
{
  for(i=0;i<180;i++)
  {
    myservo.write(i);
  }
  delay(2000);
  for(i=180;i>0;i--)
  {
    myservo.write(i);
  }
  delay(2000);
}
