# Arduino codes folder


# // printer_arduino
This code is designed to control the stepper motors of a 3D printer on each axis (X, Y, and Z).

The code defines the pin assignments for controlling stepper motors and other components such as heaters, fans, and sensors on the printer. (pins are found in pins.ino of Marlin environment)
The setup() function sets the relevant pins as outputs and enables the stepper motors by setting the enable pins to LOW (LOW makes it move here).
The loop() function demonstrates moving the stepper motor by 45 degrees, then waits for 1 second before repeating. The moveStepper() function moves the stepper motor by the specified angle.



# // printer_arduino_ROS.ino
This code works with the ROS C++ code 'move_printer'. 

This Arduino sketch is designed to control stepper motors on the X, Y, and Z axes of a 3D printer using ROS (Robot Operating System) for communication. 
It includes the necessary ROS libraries and defines global variables and constants for controlling the stepper motors and defines the pins for controlling the stepper motors on the X, Y, and Z axes.

It also defines callback functions pwmx, pwmy, and pwmz to handle incoming ROS messages for each axis. These functions set the desired angle and flag for each axis when a message is received.The code subscribes to ROS topics stepperx, steppery, and stepperz with the corresponding callback functions.

The constant 17.77 is arbitrarily set up, it can be changed. I took it from an Arduino code on a GitHub.



# // servo_arduino.ino
This code allows to control a micro servo SG90. It was to test the libraries provided by arduino and it worked.



# // stepper_arduino.ino
Before starting to work on 3 stepper motors for the printer, I began to manipulate 1 stepper motor with a separate assembly (Arduino board AtMega2560 and a Pololu md34b driver). This enabled me to understand how a stepper motor is connected to the arduino. This code allows you to control it by imposing an angle.



# // stepper_arduino_ROS.ino
It's an improved version of stepper_arduino.ino.  

It controls the stepper motor with the ROS Gazebo simulation. I have added a subscriber to retrieve the ekf data (only the y-axis data, as there is only one stepper motor and therefore only one degree of freedom). Then I calibrated the synchronization between the two so that the stepper motor moves forward at the same time as the simulated astrobee, and makes a revolution when the robot moves one meter.






