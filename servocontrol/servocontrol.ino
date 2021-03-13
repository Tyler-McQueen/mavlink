#include <Servo.h>

Servo servoarm1;
Servo servoarm2;
Servo servoarm3;
Servo servoarm4;

int inPin = 7;
int val = 0;

void setup() {
  pinMode(inPin, INPUT);
  
  servoarm1.write(90);
  servoarm2.write(90);
  servoarm3.write(90);
  servoarm4.write(90);
  
  servoarm1.attach(20);
  servoarm2.attach(21);
  servoarm3.attach(22);
  servoarm4.attach(23);
}

void loop() {
  val = digitalRead(inPin);
  if (val == HIGH) {         // check if the input is HIGH (button released)
    servoarm1.write(90);
    servoarm2.write(90);
    servoarm3.write(90);
    servoarm4.write(90);
  } else {
    servoarm1.write(180);
    servoarm2.write(180);
    servoarm3.write(180);
    servoarm4.write(180);
  }
}
