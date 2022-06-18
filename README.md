# Forwards kinematics of steering-wheels robots

In this project, a map based simulator is improved for ROS2 to simulate two types of robots : (1,1) robot and (1,2) robot, both of which are steering wheeled robots.
Intermediary nodes have been developed that will receive the control inputs (which are the forward and the steering velocities) and these inputs are transformed into the necessary twist according to the kinematic model of the desired robot and published. 
The resulting joint state positions of the robots are also published in accordance with the control inputs.
These robots will be displayed on the map and we can control them with the sliders.

To do basic tests run ```ros2 launch mapsim_steering simulation_launch.py``` 

Parameters that can be updated in the launch file
    - `robot`: `bike` or `twosteering`
    - `jsp`: directly control the joints (to test a URDF)
    - `cmd`: to use a slider GUI to control the high-level control

To run a specific robot, run 

```ros2 launch mapsim_steering simulation_launch.py robot:=bike``` for the (1,1) robot (or) 

```ros2 launch mapsim_steering simulation_launch.py robot:=two_steering``` for the (1,2) robot  

The high-level command is published on `cmd` while the low-level twist is subscribed on `cmd_vel`. 
These topics are relative to the `robot` namespace.

This node requires map_simulator and simple_launch packages.


Extension -

Static feedback control law for the (1,1) robot.

A reference trajectory would be generated and the robot would have to follow this trajectory. For this, two new nodes have been created, namely "static_feedback" and "static_kinematics".
The static_feedback node will have the trajectory generation and implementation of the static feedback control law. It will subscribe to the ‘pose' of the robot from the simulator and it will publish the v_control and beta_dot_control messages to the static_kinematics node. 
The static_kinematics node would subscribe to these messages, do the appropriate computations, and generate the relevant twist and joint state messages.
A separate launch file has been created for this.

To run the static feedback -
```ros2 launch mapsim_steering static_simulation_launch.py ```

