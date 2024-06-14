# Auto_nav package

The primary aim of the auto_nav package is to develop codes to control the astrobee without using teleop_tool commands (NASA commands to move the astrobee: see 
https://nasa.github.io/astrobee/v/develop/teleop.html), which don't represent reality. Secondly, C++ codes were created to control the MakerGear M2 3D Printer, using G-codes and Arduino.

This package includes python and C++ codes for controlling the astrobee robot with the FAM control system, and for controlling the 3D printer according to the astrobee's movement in simulation.
There are .md files inside each folder to explain what the codes are for, how they work and how to run them with ROS. 

It contains a cfg folder for setting up dynamic reconfiguration (for a specific node inside the scripts folder), a scripts folder containing all python codes, a src folder containing all C++ codes and a srv folder for service files.



# Installation

This package was created on ROS Noetic, on Ubuntu 20.04. I installed the [Astrobee](https://github.com/nasa/astrobee) simulator following the instructions for [General Users](https://nasa.github.io/astrobee/html/install-nonNASA.html).

I also work on Arduino IDE_2.3.2_Linux_64bit, which is compatible with ROS Noetic. To make a connection between Arduino and ROS, I downloaded rosserial-arduino librairies.

To understand the logics of the Arduino board AtMEGA2560 RAMBO of the printer, I downloaded the Marlin environment.



# Control the astrobee in the ISS on Gazebo using ROS

The ROS nodes written in python, inside the scripts folder of auto_nav, use the FAM (Force Allocation Module) to move. The code auto_nav.py is the main code and control the astrobee Honey which is here the default robot (it spawns in the ISS when we launch the simulator).

1. Run the Astrobee simulator
``` 
roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
```
2. Initiate the robot (because the auto_nav.py node does not work without it)
```
rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 -0.1 0"
```
3. Run the auto_nav node to move the astrobee to the desired position and orientation (choose this position and the orientation before launching the node)
```
rosrun auto_nav auto_nav.py
```

The astrobee moves without the teleop tools, using only forces and torques. This is not as accurate as the teleop tools but it is a better representation of reality. Improvements can be done, for example implementing a PID controller for more accuracy.

This auto_nav.py code is rewritten in C++ and it is in the source folder `src`.



# Control a 3D printer platform MakerGear M2 with Arduino and ROS

The main code is in the src folder in auto_nav and is named move_printer.cpp. It is connected with an Arduino code named printer_arduino_ROS.ino.

1. Power and connect the printer to the computer.

2. Upload printer_arduino_ROS.ino on Arduino software

3. Launch the Atrobee simulator
```
roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
```
4. Create a connection with rosserial
```
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=921600
```
5. Run the ROS node
```
rosrun auto_nav move_printer
```
6. Move the Astrobee with teleop
```
rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 2 0"   
```
Values must remain within the ranges, otherwise there may be a collision with the iss walls and the simulation will not work: 
X in [-0.8;0.8], Y in [-3;3] and Z in [-0.8;0.8]



# Serial communication

      ------------------
      Astrobee Simulator
           Gazebo
            EKF
      ------------------
              |
              | /honey/gnc/ekf
              | 
              V
      ------------------
       ROS node
        move_printer.cpp

      ------------------
         |   |    |
         |   |    | /stepperx, /steppery, /stepperz
         |   |    |
         V   V    V
      ------------------
      Arduino code
       printer_arduino_ROS.ino
       
      ------------------
             |
             |   Serial connection: /dev/ttyACM0
             |   Baud rate: 921600
             V
      ------------------
      3D printer
       Stepper Motors
       
      ------------------
      
Gazebo (Simulation) --> [ /honey/gnc/ekf ] --> ROS Node (Transform EKF data)

ROS node:
  - Subscribes to: /honey/gnc/ekf
  - Transforms EKF data to angles (anglex, angley, anglez)
  - Publishes to: /stepperx, /steppery, /stepperz

Arduino: 
  - Serial Connection: /dev/ttyACM0, Baud rate: 921600
  - Subscribes to: /stepperx, /steppery, /stepperz
  - Sends commands to stepper motors

Stepper Motors
  - Receive commands and move accordingly



# Inside this Jeremy folder

You can find photos and videos about the simulations, there are also images and diagrams.

There are powerpoints presenting the work carried out and the progress made during the course.

Inside the Arduino code folder, there are other codes that helped me understand how to use the Arduino software. These codes are presented and explained in a .md file.

























