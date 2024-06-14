#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

// Variable to store the received angle
signed int angle;

// Pin definitions
const int stepPin = 4;  // Pin to control the steps of the stepper motor
const int dirPin = 5;   // Pin to set the direction of the stepper motor
int x;  // Loop counter for step generation
int ENABLE = 7;  // Pin to enable the stepper driver
int DMODE2 = 6;  // Pin for DMODE2, controlling microstepping mode
int DMODE1 = 3;  // Pin for DMODE1, controlling microstepping mode
int DMODE0 = 2;  // Pin for DMODE0, controlling microstepping mode

// ROS node handle
ros::NodeHandle nh;

// Callback function to process received angle commands
// Inputs: const std_msgs::Int16& cmd_msg (ROS message containing the angle)
// Outputs: None
void pwm(const std_msgs::Int16& cmd_msg) {
  angle = cmd_msg.data;  // Retrieve the angle from the message
  if (angle < 0) {
    cw(angle);  // Move clockwise if the angle is negative
  } else {
    ccw(angle);  // Move counterclockwise if the angle is positive
  }
}

// ROS subscriber to receive angle commands on the "servo" topic
ros::Subscriber<std_msgs::Int16> sub("servo", pwm);

void setup() {
  // Configures the specified pins to act as outputs
  pinMode(ENABLE, OUTPUT);
  pinMode(DMODE2, OUTPUT);
  pinMode(DMODE1, OUTPUT);
  pinMode(DMODE0, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Initialize the ROS node
  nh.initNode();

  // Subscribe to the "servo" topic
  nh.subscribe(sub);
}

void loop() {
  // Set the microstepping mode to 1/16
  digitalWrite(DMODE2, HIGH);
  digitalWrite(DMODE1, HIGH);
  digitalWrite(DMODE0, LOW);

  // Enable the stepper driver
  digitalWrite(ENABLE, HIGH);

  // Handle any incoming ROS messages
  nh.spinOnce();
}

// Function to move the stepper motor clockwise
// Inputs: int& ang (angle to move, passed by reference)
// Outputs: None
void cw(int& ang) {
  digitalWrite(dirPin, LOW);  // Set the direction to clockwise
  for (x = 0; x < 1.8 * 1.2 * (ang * -1); x++) {
    digitalWrite(stepPin, HIGH);  // Generate a step pulse
    delayMicroseconds(500);  // Wait for 500 microseconds
    digitalWrite(stepPin, LOW);  // End the step pulse
    delayMicroseconds(500);  // Wait for 500 microseconds
  }
}

// Function to move the stepper motor counterclockwise
// Inputs: int& ang (angle to move, passed by reference)
// Outputs: None
void ccw(int& ang) {
  digitalWrite(dirPin, HIGH);  // Set the direction to counterclockwise
  for (x = 0; x < 1.8 * 1.2 * ang; x++) {
    digitalWrite(stepPin, HIGH);  // Generate a step pulse
    delayMicroseconds(500);  // Wait for 500 microseconds
    digitalWrite(stepPin, LOW);  // End the step pulse
    delayMicroseconds(500);  // Wait for 500 microseconds
  }
}
