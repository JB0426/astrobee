#include <Servo.h>

// Create a Servo object to control a servo motor
Servo myservo;

void setup() {
  // put your setup code here, to run once:

  // Attach the servo on pin 4 to the Servo object
  myservo.attach(4);
  
  // Set the initial position of the servo to 0 degrees
  myservo.write(0);
  
  // Wait for 2 seconds to allow the servo to reach the initial position
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Move the servo to 90 degrees
  myservo.write(90);
  
  // Wait for 1 second to allow the servo to reach the position
  delay(1000);
  
  // Move the servo back to 0 degrees
  myservo.write(0);
  
  // Wait for 1 second to allow the servo to reach the position
  delay(1000);
}
