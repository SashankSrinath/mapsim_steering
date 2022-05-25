# Forwards kinematics of steering-wheels robots - (1,1) robot

To do basic tests run ```ros2 launch mapsim_steering simulation_launch.py``` 

Parameters that can be updated in the launch file
    - `robot`: `bike` or `twosteering`
    - `jsp`: directly control the joints (to test a URDF)
    - `cmd`: to use a slider GUI to control the high-level control

To run a specific robot, run 

```ros2 launch mapsim_steering simulation_launch.py robot:=bike``` for the(1,1) robot (or) 

```ros2 launch mapsim_steering simulation_launch.py robot:=two_steering``` for the (1,2) robot  

The high-level command is published on `cmd` while the low-level twist is subscribed on `cmd_vel`. 
These topics are relative to the `robot` namespace.

This node requires map_simulator and simple_launch packages.
