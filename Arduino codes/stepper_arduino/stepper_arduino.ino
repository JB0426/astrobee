// Arbitrary pin values for stepper motor control
int DMODE0 = 8;  // Pin for DMODE0, controlling microstepping mode
int DMODE1 = 7;  // Pin for DMODE1, controlling microstepping mode
int DMODE2 = 6;  // Pin for DMODE2, controlling microstepping mode
int ENABLE = 9;  // Pin to enable the stepper driver
int dirPin = 5;  // Pin to set the direction of the stepper motor
int stepPin = 4; // Pin to control the steps of the stepper motor

void setup() {
  // Configures the specified pins to act as outputs
  pinMode(DMODE0, OUTPUT);
  pinMode(DMODE1, OUTPUT);
  pinMode(DMODE2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop() {
  // Sets the microstepping mode: full step microstepping
  digitalWrite(DMODE0, LOW);
  digitalWrite(DMODE1, LOW);
  digitalWrite(DMODE2, HIGH);
  
  // Enables the stepper driver at full power
  analogWrite(ENABLE, 255);

  // Generate 100 step pulses
  for (int x = 0; x < 100; x++) {
    digitalWrite(stepPin, HIGH);  // Generates a step pulse
    delay(10);                    // Waits for 10 milliseconds
    digitalWrite(stepPin, LOW);   // Ends the step pulse
  }

  delay(500);  // Waits for 500 milliseconds before changing direction

  // Changes the direction of the stepper motor
  digitalWrite(dirPin, HIGH);
}
