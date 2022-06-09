from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('robot', default_value='bike')
    sl.declare_arg('cmd', default_value=True)
    sl.declare_arg('jsp', default_value=False)
    
    sl.declare_arg('map', default_value=sl.find('map_simulator', 'house.yaml'))
    sl.declare_arg('max_height', default_value=800)
    sl.declare_arg('max_width', default_value=1200)
    sl.declare_arg('rate', default_value=20)
    sl.declare_arg('map_server', default_value=False)
    
    node_params = {}
    node_params['map'] = sl.arg('map')
    node_params['max_height'] = sl.arg('max_height')
    node_params['max_width'] = sl.arg('max_width')
    node_params['rate'] = sl.arg('rate')
    
    sl.node('map_simulator', 'simulator', parameters = node_params, output='screen')
	
    with sl.group(ns=sl.arg('robot')):  
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True, 'force_scanner' : True})
        sl.include('mapsim_steering', 'steering_launch.py', launch_arguments=sl.arg_map(('robot', 'cmd', 'jsp')))
        
    sl.node('rviz2', 'rviz2', arguments=['-d', sl.find('mapsim_steering', 'config.rviz')],
            remappings={'/robot_description': sl.name_join('/', sl.arg('robot'), '/robot_description')})
    
    sl.node('tf2_ros', 'static_transform_publisher', 'footprint', 
            arguments=['0','0','0','0','0','0',sl.name_join(sl.arg('robot'), '/base_footprint'), 'base_footprint'])

	
    return sl.launch_description()
