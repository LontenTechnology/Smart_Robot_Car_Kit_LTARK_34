#include <IRremote.h>
#define ENA  5              //pin of controlling speed---- ENA of motor driver board
#define ENB  6              //pin of controlling speed---- ENB of motor driver board
int pinLB=13;              //pin of controlling turning---- IN1 of motor driver board
int pinLF=12;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=4;            //pin of controlling turning---- IN4 of motor driver board
int RECV_PIN = 2;
IRrecv irrecv(RECV_PIN);
decode_results results;
#define IR_Go      0x00FF18E7
#define IR_Back    0x00FF4AB5
#define IR_Left    0x00FF10EF
#define IR_Right   0x00FF5AA5
#define IR_Stop    0x00FF38C7

void setup() {
  //set up motors
  pinMode(pinLB,OUTPUT);  // pin 13
  pinMode(pinLF,OUTPUT);  // pin 12
  pinMode(pinRB,OUTPUT);  // pin 7
  pinMode(pinRF,OUTPUT);  // pin 4
  pinMode(ENA,OUTPUT);    // pin 5(PWM) 
  pinMode(ENB,OUTPUT);    // pin 6(PWM) 
  Set_Speed(150);//setting speed 150
  irrecv.enableIRIn(); // Start the receiver
  Serial.begin(9600);   //initializing serial port, Bluetooth used as serial port, setting baud ratio at 9600 
  stopp();             //Initialize the car status and stop  
}

void loop() {
    IR_Control();
}
void IR_Control(void)
{
  unsigned long Key;
  if(irrecv.decode(&results)) //judging if serial port receives data   
  {
    Key = results.value;
    switch(Key)
     {
       case IR_Go:advance();   //going forward
       break;
       case IR_Back: back();   //back
       break;
       case IR_Left:turnL();   //left    
       break;
       case IR_Right:turnR(); //right
       break;
       case IR_Stop:stopp();   //stop
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
   } 
  
}

void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(ENA,pwm);
  analogWrite(ENB,pwm);
}
void advance()    //  going forward
    {
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
   
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
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,LOW);
    
    }
void back()         //back up
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //making motor move towards left rear
     digitalWrite(pinLF,LOW);
    }



