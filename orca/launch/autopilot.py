from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    scan_topic='/projected_scan'
    
    return LaunchDescription([
        
        Node(
            package='orca',
            executable='pose',
            name='pose'
        ),
        
        
        Node(
            package='orca',
            executable='autopilot',
            name='autopilot'
        ),
        
        
        Node(
            package='orca',
            executable='motion',
            name='motion'
        ),
        
    ])
