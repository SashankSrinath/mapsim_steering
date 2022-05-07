# Forwards kinematics of steering-wheels robots

To do basic tests run ```ros2 launch mapsim_steering simulation_launch.py``` with parameters:
    - `robot`: `bike` or `twosteering`
    - `jsp`: directly control the joints (to test a URDF)
    - `cmd`: to use a slider GUI to control the high-level control
    
The high-level command is publishe on `cmd` while the low-level twist is subscribed on `cmd_vel`. 
These topics are relative to the `robot` namespace.
