#include <FastLED.h>                                      //Define the RGB driver chip WS2812 library
#define LED_PIN     A4                                    //The ARDUINO driver pin is A4
#define NUM_LEDS    6                                     //RCWL-1633 is equipped with 6 series connected RGB light beads
CRGB leds[NUM_LEDS];
float       distance;                                     //Define distance data
const int   I_O = A5;                                     //I/O connected to pin A5, single bus ranging

void setup()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(9600);                                     //Baud rate 9600
  pinMode(I_O, INPUT);                                    //Set I_O as input pin
  Serial.println(" Distance measurement begins：");
}

void loop()
{
  pinMode(I_O, OUTPUT);
  digitalWrite(I_O, HIGH);
  delayMicroseconds(100);
  digitalWrite(I_O, LOW);
  pinMode(I_O, INPUT);
  distance  = pulseIn(I_O, HIGH, 500000);
  distance = distance * 340 / 2 / 10000;
  Serial.print(distance);
  Serial.println("CM");
  pinMode(I_O, OUTPUT);
  digitalWrite(I_O, LOW);
  if (distance <= 10)//Emit red light
  {
    leds[0] = CRGB(255, 0, 0);
    leds[1] = CRGB(255, 0, 0);
    leds[2] = CRGB(255, 0, 0);
    leds[3] = CRGB(255, 0, 0);
    leds[4] = CRGB(255, 0, 0);
    leds[5] = CRGB(255, 0, 0);
    FastLED.show();
  }
  else if ((10 < distance) && (distance <= 20))//Emit green light
  {
    leds[0] = CRGB(0, 255, 0);
    leds[1] = CRGB(0, 255, 0);
    leds[2] = CRGB(0, 255, 0);
    leds[3] = CRGB(0, 255, 0);
    leds[4] = CRGB(0, 255, 0);
    leds[5] = CRGB(0, 255, 0);
    FastLED.show();
  }
  else//Emit blue light
  {
    leds[0] = CRGB(0, 0, 255);
    leds[1] = CRGB(0, 0, 255);
    leds[2] = CRGB(0, 0, 255);
    leds[3] = CRGB(0, 0, 255);
    leds[4] = CRGB(0, 0, 255);
    leds[5] = CRGB(0, 0, 255);
    FastLED.show();
  }
  delay(100);
}
