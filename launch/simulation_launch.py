from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('robot', default_value='bike')
    sl.declare_arg('cmd', default_value=True)
    sl.declare_arg('jsp', default_value=True)
    
    sl.node('map_simulator', 'simulator', parameters = {'map': sl.find('mapsim_steering', 'void.yaml'),
                                                        'display': False})
	
    with sl.group(ns=sl.arg('robot')):  
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True})
        sl.include('mapsim_steering', 'steering_launch.py', launch_arguments=sl.arg_map(('robot', 'cmd', 'jsp')))
        
    sl.node('rviz2', 'rviz2', arguments=['-d', sl.find('mapsim_steering', 'config.rviz')],
            remappings={'/robot_description': sl.name_join('/', sl.arg('robot'), '/robot_description')})
    
    sl.node('tf2_ros', 'static_transform_publisher', 'footprint', 
            arguments=['0','0','0','0','0','0',sl.name_join(sl.arg('robot'), '/odom'), 'base_footprint'])

	
    return sl.launch_description()
