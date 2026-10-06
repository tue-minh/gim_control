import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gim_control')
    
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'display.launch.py')
        )
    )
    
    gim_control_node = Node(
        package='gim_control',
        executable='gim_control_node',
        name='gim_control_node',
        output='screen'
    )
    
    return LaunchDescription([
        display_launch,
        gim_control_node
    ])
