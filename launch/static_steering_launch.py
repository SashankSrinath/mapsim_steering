from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system
import sys

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('robot', default_value='bike')
    sl.declare_arg('jsp', default_value=False)
    sl.declare_arg('cmd', default_value=False)
    
    sl.robot_state_publisher('mapsim_steering', sl.name_join(sl.arg('robot'), '.urdf'), 'urdf')

    sl.node('mapsim_steering', 'static_kinematics')
    sl.node('mapsim_steering', 'static_feedback') 
    
    with sl.group(if_arg='jsp'):
        sl.joint_state_publisher()
        
    with sl.group(if_arg='cmd'):
        sl.node('slider_publisher', 'slider_publisher', 
                arguments=[sl.find('mapsim_steering', sl.name_join(sl.arg('robot'), '.yaml'), 'launch')])
       
    return sl.launch_description()
