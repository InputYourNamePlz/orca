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
            name='pose',
            remappings=[
                ('/scan', scan_topic)
            ]
        ),
        
        
        Node(
            package='orca',
            executable='autopilot',
            name='autopilot',
            remappings=[
                ('/scan', scan_topic)
            ]
        ),
        
        
        Node(
            package='orca',
            executable='motion',
            name='motion',
            remappings=[
                ('/scan', scan_topic)
            ]
        ),
        
    ])