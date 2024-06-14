# cfg folder


# // AutoNavConfig.cfg

This code sets up the dynamic reconfigure for the auto_nav_dyn.py.

```
gen.add("desired_position_x", double_t, 0, "Desired position along X axis", 11.0, -4.5, 11.5)
gen.add("desired_position_y", double_t, 0, "Desired position along Y axis", -6.0, -10.0, 7.0)
gen.add("desired_position_z", double_t, 0, "Desired position along Z axis", 5, 4.0, 6.0)
```
The parameters desired_position_x, desired_position_y, and desired_position_z represent the desired positions of the Astrobee robot along the X, Y, and Z axes inside the ISS simulation in Gazebo.

    Default Value: The initial position value set when the node starts. For example, 11.0 for desired_position_x is the default position along the X axis.
    Minimum Value: The minimum boundary value that the Astrobee can move to within the ISS. For desired_position_x, -4.5 is the minimum X position.
    Maximum Value: The maximum boundary value that the Astrobee can move to within the ISS. For desired_position_x, 11.5 is the maximum X position.

These minimum and maximum values are determined based on the size and boundaries of the ISS environment in the Gazebo simulation. They ensure that the Astrobee operates within the designated space of the ISS.

For example:

    desired_position_x: Default is 11.0, minimum is -4.5, and maximum is 11.5.
    desired_position_y: Default is -6.0, minimum is -10.0, and maximum is 7.0.
    desired_position_z: Default is 5.0, minimum is 4.0, and maximum is 6.0.

These settings allow the user to dynamically adjust the desired positions of the Astrobee while ensuring it stays within the operational boundaries of the ISS simulation environment.
