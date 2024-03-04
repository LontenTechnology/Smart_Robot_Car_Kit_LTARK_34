#define ENA  5              //pin of controlling speed---- ENA of motor driver board
#define ENB  6              //pin of controlling speed---- ENB of motor driver board
int pinLB = 13;            //pin of controlling turning---- IN1 of motor driver board
int pinLF = 12;           //pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;          //pin of controlling turning---- IN3 of motor driver board
int pinRF = 4;          //pin of controlling turning---- IN4 of motor driver board
void Infrared_Tracing() {
  int Left_Tra_Value = 1;
  int Center_Tra_Value = 1;
  int Right_Tra_Value = 1;
  int Black = 1;
  Left_Tra_Value = digitalRead(9);
  Center_Tra_Value = digitalRead(10);
  Right_Tra_Value = digitalRead(11);
  if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {//010
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
}

void setup() {
  Serial.begin(9600);
  pinMode(pinLB, OUTPUT); // pin 13
  pinMode(pinLF, OUTPUT); // pin 12
  pinMode(pinRB, OUTPUT); // pin 7
  pinMode(pinRF, OUTPUT); // pin 4
  pinMode(ENA, OUTPUT);   // pin 5(PWM)
  pinMode(ENB, OUTPUT);   // pin 6(PWM)
  Set_Speed(150);//setting speed 150
  pinMode(9, INPUT);//Left line tracking sensor is connected to the digital IO port D9
  pinMode(10, INPUT);//Center line patrol sensor is connected to the digital IO port D10
  pinMode(11, INPUT);//Right line tracking sensor is connected to the digital IO port D11
}

void loop() {
  Infrared_Tracing();

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

