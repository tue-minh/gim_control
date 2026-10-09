import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gim_control')
    
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'gim_arm.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    with open(default_urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            name='urdf_model', 
            default_value=default_urdf_path, 
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rviz_config', 
            default_value=default_rviz_config_path, 
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false', 
            description='Use simulation (Gazebo) clock if true'),
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
