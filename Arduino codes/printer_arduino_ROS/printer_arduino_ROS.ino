#include <ros.h>
#include <std_msgs/Int8.h>

// Initialize ROS node handle
ros::NodeHandle nh;

// Define pin assignments for the stepper motors
// Pins are taken from the pins.ino of the Marlin environment as RAMBO arduino board
#define X_STEP_PIN 37
#define X_DIR_PIN 48
#define X_ENABLE_PIN 29
#define Y_STEP_PIN 36
#define Y_DIR_PIN 49
#define Y_ENABLE_PIN 28
#define Z_STEP_PIN 35
#define Z_DIR_PIN 47
#define Z_ENABLE_PIN 27

// Global variables to store the target angles and movement flags
volatile int anglex = 0;
volatile int angley = 0;
volatile int anglez = 0;
volatile bool move_x = false;
volatile bool move_y = false;
volatile bool move_z = false;

// Callback function for X-axis stepper motor command
// Inputs: cmd_msg - message containing the angle to move
void pwmx(const std_msgs::Int8& cmd_msg) {
  anglex = cmd_msg.data;
  move_x = true;
}

// Callback function for Y-axis stepper motor command
// Inputs: cmd_msg - message containing the angle to move
void pwmy(const std_msgs::Int8& cmd_msg) {
  angley = cmd_msg.data;
  move_y = true;
}

// Callback function for Z-axis stepper motor command
// Inputs: cmd_msg - message containing the angle to move
void pwmz(const std_msgs::Int8& cmd_msg) {
  anglez = cmd_msg.data;
  move_z = true;
}

// ROS subscribers for each stepper motor
ros::Subscriber<std_msgs::Int8> subx("stepperx", pwmx);
ros::Subscriber<std_msgs::Int8> suby("steppery", pwmy);
ros::Subscriber<std_msgs::Int8> subz("stepperz", pwmz);

void setup() {
  // Set pin modes for the stepper motor control pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  // Initialize ROS node and subscribe to topics
  nh.initNode();
  nh.subscribe(subx);
  nh.subscribe(suby);
  nh.subscribe(subz);
}

void loop() {
  // Process incoming ROS messages
  nh.spinOnce();
  delay(10);

  // Check if a movement command was received for X-axis
  if (move_x) {
    moveStepper(anglex, X_STEP_PIN, X_DIR_PIN);
    move_x = false; // Reset the flag after moving
  }

  // Check if a movement command was received for Y-axis
  if (move_y) {
    moveStepper(angley, Y_STEP_PIN, Y_DIR_PIN);
    move_y = false; // Reset the flag after moving
  }

  // Check if a movement command was received for Z-axis
  if (move_z) {
    moveStepper(anglez, Z_STEP_PIN, Z_DIR_PIN);
    move_z = false; // Reset the flag after moving
  }
}

// Function to move the stepper motor by a given angle
// Inputs: angle - the angle in degrees to move the motor
//         step_pin - the pin controlling the step signal
//         dir_pin - the pin controlling the direction signal
void moveStepper(int angle, int step_pin, int dir_pin) {
  int steps = abs(angle) * 17.77;  // Calculate the number of steps for the given angle
  digitalWrite(dir_pin, angle > 0 ? HIGH : LOW);  // Set the direction based on the sign of the angle

  // Generate the required number of step pulses
  for (int i = 0; i < steps; ++i) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(1000); // Adjust delay to control speed of stepping
    digitalWrite(step_pin, LOW);
    delayMicroseconds(1000); // Adjust delay to control speed of stepping
  }
}



