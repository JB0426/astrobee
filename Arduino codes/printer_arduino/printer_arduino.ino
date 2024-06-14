// Define pin assignments for the stepper motors and other peripherals
// Pins are taken from the pins.ino of the Marlin environment as RAMBO arduino board
#define X_STEP_PIN 37
#define X_DIR_PIN 48
#define X_MIN_PIN 12
#define X_MAX_PIN -1 // Disabled, pin 24 repurposed for ISR signal
#define X_ENABLE_PIN 29
#define X_MS1_PIN 40
#define X_MS2_PIN 41

#define Y_STEP_PIN 36
#define Y_DIR_PIN 49
#define Y_MIN_PIN 11
#define Y_MAX_PIN -1 // Disabled, pin 23 repurposed for ISR signal
#define Y_ENABLE_PIN 28
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39

#define Z_STEP_PIN 35
#define Z_DIR_PIN 47
#define Z_MIN_PIN 10
#define Z_MAX_PIN 30
#define Z_ENABLE_PIN 27
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67

#define ISR_signal 23 // Set pin 23 (originally Y_MAX pin) for ISR signal
#define LCD_signal 24 // Set pin 24 (originally X_MAX pin) for LCD update signal

// Constants for stepper motor control
const int stepsPerRevolution = 200; // Change according to your motor's steps per revolution
const int anglePerStep = 360 / stepsPerRevolution; // Calculate angle per step

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
  pinMode(Z_ENABLE_PIN, OUTPUT);

  // Enable the stepper motors by setting the enable pins to LOW
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
}

void loop() {
  // Example: Move the Z-axis stepper motor by 45 degrees
  moveStepper(45);
  delay(1000); // Wait for 1 second before repeating
}

// Function to move the stepper motor by a given angle
// Inputs: angle - the angle in degrees to move the motor
void moveStepper(int angle) {
  int steps = angle / anglePerStep; // Calculate the number of steps needed for the given angle
  
  // Set the direction for the Z-axis motor
  digitalWrite(Z_DIR_PIN, HIGH);

  // Loop to generate the required number of steps
  for (int i = 0; i < steps; i++) {
    // Generate a step pulse for the Z-axis motor
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(500); // Adjust delay to control speed of stepping
    digitalWrite(Z_STEP_PIN, LOW);
    delayMicroseconds(500); // Adjust delay to control speed of stepping
  }
}






