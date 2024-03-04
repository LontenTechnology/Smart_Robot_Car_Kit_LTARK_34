#include <IRremote.h>
#define ENA  5              //pin of controlling speed---- ENA of motor driver board
#define ENB  6              //pin of controlling speed---- ENB of motor driver board
int pinLB = 13;            //pin of controlling turning---- IN1 of motor driver board
int pinLF = 12;           //pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;          //pin of controlling turning---- IN3 of motor driver board
int pinRF = 4;          //pin of controlling turning---- IN4 of motor driver board
int RECV_PIN = 2;
IRrecv irrecv(RECV_PIN);
decode_results results;
#define IR_Go      0x00FF18E7
#define IR_Back    0x00FF4AB5
#define IR_Left    0x00FF10EF
#define IR_Right   0x00FF5AA5
#define IR_Stop    0x00FF38C7
#include <Servo.h>
#include <FastLED.h>                                      //Define the RGB driver chip WS2812 library
#define LED_PIN     A4                                    //The ARDUINO driver pin is A4
#define NUM_LEDS    6                                     //RCWL-1633 is equipped with 6 series connected RGB light beads
CRGB leds[NUM_LEDS];
const int   I_O = A5;                                     //I/O connected to pin A5, single bus ranging
Servo myservo;
float  distance;
volatile int DL;
volatile int DM;
volatile int DR;
int tonepin = 3; // Use interface 3

int flag=0;
unsigned char bluetooth_data; 
void M_Control_IO_config(void)
{
  pinMode(pinLB, OUTPUT); // pin 13
  pinMode(pinLF, OUTPUT); // pin 12
  pinMode(pinRB, OUTPUT); // pin 7
  pinMode(pinRF, OUTPUT); // pin 4
  pinMode(ENA, OUTPUT);   // pin 5(PWM)
  pinMode(ENB, OUTPUT);   // pin 6(PWM)
  Set_Speed(150);//setting speed 150
}

void Follow()
{
  flag = 0;
  while (flag == 0)
  {
    DM = checkdistance(); //Get the current distance
    if (DM <= 15) {
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
    if (Serial.available())
    {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;
      }
    }
  }
}

void Line_Tracking(void) //function of line tracking
{
  flag = 0;
  while (flag == 0)
  {
    int Left_Tra_Value = 1;
    int Center_Tra_Value = 1;
    int Right_Tra_Value = 1;
    int Black = 1;
    Left_Tra_Value = digitalRead(9);
    Center_Tra_Value = digitalRead(10);
    Right_Tra_Value = digitalRead(11);
    if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {//010
      digitalWrite(tonepin, HIGH); //Sound
      delay(3);
      digitalWrite(tonepin, LOW);  //Nosound
      delay(1);
      advance();
    } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {//110
      turnL();
    } else if (Left_Tra_Value == Black && (Center_Tra_Value != Black && Right_Tra_Value != Black)) {//100
      turnL();
    } else if (Left_Tra_Value != Black && (Center_Tra_Value != Black && Right_Tra_Value == Black)) {//001
      turnR();
    } else if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) {//011
      turnR();
    } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) { //111
      stopp();
    }
    if (Serial.available())
    {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;
      }
    }

  }


}








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
void Detect_obstacle_distance() {
  myservo.write(160);
  for (int i = 0; i < 3; i = i + 1) {
    DL = checkdistance();
    delay(100);
  }
  myservo.write(20);
  for (int i = 0; i < 3; i = i + 1) {
    DR = checkdistance();
    delay(100);
  }
}


void Ultrasonic_Obstacle_Avoidance()
{
  flag = 0;
  while (flag == 0)
  {
    DM = checkdistance();
    if (DM < 20) {
      green();
      stopp();
      delay(1000);
      Detect_obstacle_distance();
      if (DL < 20 || DR < 20) {
        green();
        if (DL > DR) {
          myservo.write(90);
          turnL();
          delay(200);
          advance();
        } else {
          myservo.write(90);
          turnR();
          delay(200);
          advance();
        }
      }
      if (DL < 10 || DR < 10) {
        red();
        myservo.write(90);
        back();
        delay(200);
        stopp();
      }
    }
    else {
      blue();
      myservo.write(90);
      advance();
    }

    if (Serial.available())
    {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;

      }
    }
  }
}






void Infrared_Remote_Control(void)   //remote control，when pressing“#”，it quitting from the mode
{
  flag = 0;
  while (flag == 0)
  {
    unsigned long Key;
    if (irrecv.decode(&results)) //judging if serial port receives data
    {
      Key = results.value;
      switch (Key)
      {
        case IR_Go: advance();  //going forward
          break;
        case IR_Back: back();   //back
          break;
        case IR_Left: turnL();  //left
          break;
        case IR_Right: turnR(); //right
          break;
        case IR_Stop: stopp();  //stop
          break;
        default:
          break;
      }
      irrecv.resume(); // Receive the next value
    }
  }
}




void setup()
{
  M_Control_IO_config();     //motor controlling the initialization of IO
  irrecv.enableIRIn(); // Start the receiver
  Serial.begin(9600);   //initializing serial port, Bluetooth used as serial port, setting baud ratio at 9600
  pinMode(9, INPUT);//Left line tracking sensor is connected to the digital IO port D9
  pinMode(10, INPUT);//Center line patrol sensor is connected to the digital IO port D10
  pinMode(11, INPUT);//Right line tracking sensor is connected to the digital IO port D11
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(I_O, INPUT);                                    //Set I_O as input pin
  pinMode (tonepin, OUTPUT);
  stopp();             //Initialize the car status and stop
  DL = 0;
  DM = 0;
  DR = 0;
  myservo.attach(8);
  myservo.write(90); //Initialize the servo angle to 90 degrees
}
void loop()
{
  if (Serial.available())
  {
    bluetooth_data = Serial.read();
    Serial.println(bluetooth_data);
  }
  switch (bluetooth_data) {
    case 'U':
      advance();
      break;
    case 'D':
      back();
      break;
    case 'L':
      turnL();
      break;
    case 'R':
      turnR();
      break;
    case 'S':
      stopp();
      break;
    case 'T':
      stopp();
      Line_Tracking();
      break;
    case 'O':
      stopp();
      Ultrasonic_Obstacle_Avoidance();
      break;
    case 'I':
      stopp();
      Infrared_Remote_Control();
      break;
    case 'G':
      stopp();
      Follow();
      break;
    default:
      break;
  }
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


