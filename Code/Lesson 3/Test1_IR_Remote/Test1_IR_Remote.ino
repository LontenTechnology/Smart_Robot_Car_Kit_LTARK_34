#include <IRremote.h>
 int RECV_PIN = 2;
 IRrecv irrecv(RECV_PIN);
 decode_results results;
 void setup()
{
  pinMode(RECV_PIN,INPUT);
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}
 void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
  }
}

