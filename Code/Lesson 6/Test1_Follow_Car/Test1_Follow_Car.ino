#include <FastLED.h>                                      //Define the RGB driver chip WS2812 library
#define LED_PIN     A4                                    //The ARDUINO driver pin is A4
#define NUM_LEDS    6                                     //RCWL-1633 is equipped with 6 series connected RGB light beads
CRGB leds[NUM_LEDS];
const int   I_O = A5;                                     //I/O connected to pin A5, single bus ranging
#define ENA  5              //pin of controlling speed---- ENA of motor driver board
#define ENB  6              //pin of controlling speed---- ENB of motor driver board
int pinLB = 13;            //pin of controlling turning---- IN1 of motor driver board
int pinLF = 12;           //pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;          //pin of controlling turning---- IN3 of motor driver board
int pinRF = 4;          //pin of controlling turning---- IN4 of motor driver board
float  distance;
volatile int DM;
float checkdistance() {
  pinMode(I_O, OUTPUT);
  digitalWrite(I_O, HIGH);
  delayMicroseconds(100);
  digitalWrite(I_O, LOW);
  pinMode(I_O, INPUT);
  distance  = pulseIn(I_O, HIGH, 500000);
  distance = distance * 340 / 2 / 10000;
  return distance;
}
void setup() {
  pinMode(pinLB, OUTPUT); // pin 13
  pinMode(pinLF, OUTPUT); // pin 12
  pinMode(pinRB, OUTPUT); // pin 7
  pinMode(pinRF, OUTPUT); // pin 4
  pinMode(ENA, OUTPUT);   // pin 5(PWM)
  pinMode(ENB, OUTPUT);   // pin 6(PWM)
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(9600);                                     //Baud rate 9600
  pinMode(I_O, INPUT);                                    //Set I_O as input pin
  Set_Speed(150);//setting speed 150
  DM = 0;
}
void red() {
  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  leds[3] = CRGB(255, 0, 0);
  leds[4] = CRGB(255, 0, 0);
  leds[5] = CRGB(255, 0, 0);
  FastLED.show();
}
void green() {
  leds[0] = CRGB(0, 255, 0);
  leds[1] = CRGB(0, 255, 0);
  leds[2] = CRGB(0, 255, 0);
  leds[3] = CRGB(0, 255, 0);
  leds[4] = CRGB(0, 255, 0);
  leds[5] = CRGB(0, 255, 0);
  FastLED.show();
}
void blue() {
  leds[0] = CRGB(0, 0, 255);
  leds[1] = CRGB(0, 0, 255);
  leds[2] = CRGB(0, 0, 255);
  leds[3] = CRGB(0, 0, 255);
  leds[4] = CRGB(0, 0, 255);
  leds[5] = CRGB(0, 0, 255);
  FastLED.show();
}
void loop() {
   DM=checkdistance();  //Get the current distance
   if(DM<=15){
    red();
    Serial.println("stop");
    stopp();
    delay(300);
    DM = checkdistance();
    delay(500);
  }
  else { //There are no obstacles ahead, the car is moving forward
    green();
    Serial.println("forward");
    advance();
    delay(100);
    stopp();
  }
 }
void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}
void advance()    //  going forward
{
  digitalWrite(pinRB, LOW); // making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW); // making motor move towards left rear
  digitalWrite(pinLF, HIGH);

}
void turnR()        //turning right(dual wheel)
{
  digitalWrite(pinRB, LOW); //making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW); //making motor move towards left front
}
void turnL()         //turning left(dual wheel)
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW );  //making motor move towards right front
  digitalWrite(pinLB, LOW);  //making motor move towards left rear
  digitalWrite(pinLF, HIGH);
}
void stopp()        //stop
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, LOW);

}
void back()         //back up
{
  digitalWrite(pinRB, HIGH); //making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH); //making motor move towards left rear
  digitalWrite(pinLF, LOW);
}


