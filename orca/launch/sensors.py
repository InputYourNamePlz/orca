from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='ebimu_driver',
            executable='ebimu_driver',
            name='ebimu_driver'
        ),
        
        Node(
            package='orca_localization',
            executable='scan_stabilizer',
            name='scan_stabilizer'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch.py'
                )
            ])
        )
    ])