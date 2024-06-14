# src folder

# // auto_nav.cpp

- How to launch on terminal:
       roscore
       roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
       rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 -0.1 0"  (need to initialize the astrobee otherwise this node does not work)
       rosrun auto_nav auto_nav
       
- Desciption:
It is the C++ version of auto_nav.py and works the same. 
I created a class AutoNav that handles the ROS node's functionality, position control, and FAM commands. The constructor initializes the desired position and orientation and sets up publisher and subscriber for FAM commands and EKF state updates. We find find the same functions 'calculate_commands' and the callback function for EKF.



# // auto_nav_rev1.cpp
This code is not finished. The main aim was to improve the auto_nav.cpp code by using functions from another control code 'ctl_ros.cc' supplied by NASA, in particular a function implementing a PID controller. Other functions seem to be important (ctlStep, sendControlCommand,...)



# // gcode_printer.cpp

This code works with the default Arduino code 'Marlin.ino' (need first to upload this code on Arduino before rosrun the C++ code). 'Marlin.ino' was downloaded on the printer website MakerGear M2.

- How to launch on terminal:
       roscore
       roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
       rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200   (baud speed min for the printer and Marlin)
       rosrun auto_nav gcode_printer
       rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 2 0"  (you can put anything for x, y ,z)
       
- Desciption:
I created a class PrinterController that handles the ROS node's functionality, serial communication and printer control. The constructor loads parameters for the serial and baud rate, sets up a subscriber for the EKF state updates, a serial communication callbacks for timeout and shutdown. It also sends initial commands to the printer to set up PID, home position, steps per unit, max feedrates, and max acceleration (these functions are found in the Marlin environment).



# // move_printer.cpp,   the most important here !

This code works with the Arduino code 'printer_arduino_ROS.ino' (need first to upload this code on Arduino before rosrun the C++ code). 'move_printer.cpp' and 'printer_arduino_ROS.ino' were created at the same time to allow us to control the printer with the simulation Gazebo/Rviz.

- How to launch on terminal:
       roscore
       roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
       rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=921600   (/!\ baud speed min for this code otherwise it does not work)
       rosrun auto_nav move_printer
       rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 2 0"  (you can put anything for x, y ,z)
       
- Desciption:
I created a class ArduinoROS that handles the ROS node's functionality and stepper motor control. The construtor subscribes to the EKF topic and advertizes topics for controlling the stepper motors of the printer. The 3 functions stepperMotorX, stepperMotorY and stepperMotorZ publish angle commands to respective stepper motors. The callback function handles EKF state updates, calculates position differences, converts them to angles, and sends them to the stepper motors. The main loop runs at a high frequency (20kHz) to continuously process callbacks and update motors commands













