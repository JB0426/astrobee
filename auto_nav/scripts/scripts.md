# scripts folder

# // auto_nav.py 

- How to launch on terminal:
       roscore
       roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
       rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 -0.1 0"  (need to initialize the astrobee otherwise this node does not work)
       rosrun auto_nav auto_nav.py
       
- Desciption:

```
desired_position = Vector3(x=11.0, y=-5.0, z=4.9)
```
Before starting the simulation, you may need to change the desired position of the Astrobee robot. The position is represented as a Vector3 object with x, y, and z coordinates. In practice, it is easier to adjust the y value because the x and z coordinates are more constrained by the ISS environment.



The code initializes a ROS publisher (rospy.Publisher) to send FAM (Force and Moment) commands to control the Astrobee's movement. These commands are published to the topic "/honey/gnc/ctl/command".The messages being published are of type FamCommand.
To calculate the FAM commands accurately, the code sets up a ROS subscriber (rospy.Subscriber) to receive Extended Kalman Filter (EKF) state updates from the Astrobee.The EKF state messages are received from the topic "/honey/gnc/ekf", and they are of type EkfState.
```
pub = rospy.Publisher("/honey/gnc/ctl/command", FamCommand, queue_size=1)
rospy.Subscriber("/honey/gnc/ekf", EkfState, callback)
```
There is a callback function (callback) defined to handle the received EKF state messages. This function processes the EKF state data and likely updates variables or triggers actions needed for calculating FAM commands.
Additionally, there's a function (calculate_commands()) mentioned, which presumably performs the calculations based on the received EKF data and publishes the resulting FAM commands using the publisher.



# //  auto_nav_dyn.py

- How to launch on terminal:
       roscore
       roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
       rosrun executive teleop_tool -ns "honey" -move -relative -pos "0 -0.1 0"  (need to initialize the astrobee otherwise this node does not work)
       rosrun auto_nav auto_nav_dyn.py
       rosrun rqt_gui rqt_gui -s reconfigure  (launch the dynamic reconfigure interface where you can control by yourself the robot without stopping the simulation)
      

- Desciption:      
       
This code represents an improvement over the previous version. I integrated the dynamic reconfigure tool into the auto_nav.py code to enhance flexibility. With this addition, we are no longer constrained to stopping and restarting the code every time we want to change the value of the desired position. Instead, we can dynamically adjust the desired position parameters while the code is running, providing a more efficient and convenient way to control the behavior of the Astrobee robot.
       
```
from dynamic_reconfigure.server import Server
from auto_nav.cfg import AutoNavConfigConfig as AutoNavConfig

def reconfigure_callback(config, level):
    global desired_position
    desired_position.x = config['desired_position_x']
    desired_position.y = config['desired_position_y']
    desired_position.z = config['desired_position_z']
    return config
    
srv = Server(AutoNavConfig, reconfigure_callback)
```
These lines have been added to the code to set up dynamic reconfigure functionality during the simulation launch. The AutoNavConfig.cfg file, described inside the cfg.md file, is utilized for this purpose. Additionally, a new callback function (reconfigure_callback) has been implemented.

When these lines are executed, the Server class from the dynamic_reconfigure.server module is instantiated with AutoNavConfig and reconfigure_callback. This sets up the dynamic reconfigure server, enabling parameters defined in AutoNavConfig.cfg to be dynamically adjusted during the simulation.

The reconfigure_callback function is responsible for updating the desired_position variable with the new values obtained from the dynamic reconfigure parameters. This allows for real-time adjustments to the desired position of the Astrobee robot without the need to restart the simulation.
    
       
       
# // bumble_auto_nav.py (4 codes) and queen_auto_nav.py

- How to launch on terminal:
       roscore
       roslaunch astrobee sim.launch default_robot:=false dds:=false robot:=sim_pub rviz:=true sviz:=true
       rosrun executive teleop_tool -ns "bumble" -move -relative -pos "0 -0.1 0"  (or queen)
       rosrun auto_nav bumble_auto_nav.py   (or queen_auto_nav.py)
       
- Desciption:

These scripts, while similar to auto_nav.py, serve to control specific Astrobee robots (Honey, Bumble, and Queen) by subscribing to the EKF topic and publishing FAM commands. Notably, they are not updated and represent older versions of auto_nav.py. However, despite being older versions, they are still functional and capable of controlling their respective robots effectively. Each script is tailored to work with a specific robot, allowing for simultaneous launch of multiple robots. Due to the presence of namespaces (ns), launching all three Astrobees simultaneously requires running each script in a separate terminal.
       
       
       
# //  stop_node_service.py

- How to launch on terminal:
       rosrun auto_nav stop_node_service.py
       rosservice call /stop_node
       rosrun rosserial_python serial_node.py /dev/ttyACM0 
       
- Desciption:

This code creates a ROS service that is designed to stop another node (/honey/ctl) that interferes with the functionality of auto_nav.py. When this service is called, it executes the command to kill the /honey/ctl node, effectively stopping it from running. This service is particularly useful in scenarios where the interference of the /honey/ctl node needs to be addressed to ensure the proper operation of the auto_nav.py script.

        
       
       
       
       
       
       
       
