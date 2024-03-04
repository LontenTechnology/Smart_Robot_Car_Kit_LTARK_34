int tonepin = 3; // Use interface 3
void setup ()
{
  pinMode (tonepin, OUTPUT);
}
void loop ()
{
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(tonepin, HIGH); //Sound
    delay(3);        
    digitalWrite(tonepin, LOW);  //Nosound
    delay(1);         
  }
  delay(1000);
}
