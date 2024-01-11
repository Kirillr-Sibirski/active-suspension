#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

int buttonState1 = 0;  // variable for reading the pushbutton status
int buttonState2 = 0;  // variable for reading the pushbutton status

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo1.attach(8);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(7);  // attaches the servo on pin 9 to the servo object
  myservo3.attach(6);  // attaches the servo on pin 9 to the servo object

  pinMode(3, INPUT);
  pinMode(4, INPUT);
}

void loop() {
  buttonState1 = digitalRead(3);
  buttonState2 = digitalRead(4);

  if (buttonState1 == HIGH) { // One side up other down
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(val);                  // sets the servo position according to the scaled value
    myservo1.write(val);
    myservo2.write(val);
    myservo3.write(val);
    delay(15);                           // waits for the servo to get there
  }
  else if (buttonState2 == HIGH) { // Front up, back down
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(180-val);                  // sets the servo position according to the scaled value
    myservo1.write(val);
    myservo2.write(180-val);
    myservo3.write(val);
    delay(15);                           // waits for the servo to get there
  }
  else { // Up and down all together
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(val);                  // sets the servo position according to the scaled value
    myservo1.write(val);
    myservo2.write(180-val);
    myservo3.write(180-val);
    delay(15);                           // waits for the servo to get there
  }
}
