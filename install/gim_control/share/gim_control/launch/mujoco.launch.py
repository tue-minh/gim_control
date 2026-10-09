import os
import re
import mujoco
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gim_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'gim_arm.urdf')
    
    with open(urdf_file, 'r') as infp:
        urdf_content = infp.read()
    
    # 1. Modify URDF content for MuJoCo to read
    # Replace package:// with absolute path for MuJoCo URDF parser
    mujoco_urdf_content = urdf_content.replace('package://gim_control/', pkg_share + '/')
    
    # Update the mujoco_model parameter in the ROS2 control tag to point to our final wrapper
    wrapper_xml_path = '/tmp/gim_arm_wrapper.xml'
    if '<param name="mujoco_model">' in mujoco_urdf_content:
        mujoco_urdf_content = re.sub(r'<param name="mujoco_model">.*?</param>', f'<param name="mujoco_model">{wrapper_xml_path}</param>', mujoco_urdf_content)

    mujoco_urdf_path = '/tmp/gim_arm_mujoco.urdf'
    with open(mujoco_urdf_path, 'w') as outfp:
        outfp.write(mujoco_urdf_content)
        
    # 2. Compile URDF to MJCF using MuJoCo
    model = mujoco.MjModel.from_xml_path(mujoco_urdf_path)
    tmp_mjcf_path = '/tmp/gim_arm_converted.xml'
    mujoco.mj_saveLastXML(tmp_mjcf_path, model)
    
    # 3. Read MJCF, inject actuators, write to wrapper_xml_path
    tree = ET.parse(tmp_mjcf_path)
    root = tree.getroot()
    
    actuator = ET.SubElement(root, 'actuator')
    
    # Position Actuators
    ET.SubElement(actuator, 'position', name='base_joint', joint='base_joint', kp='100')
    ET.SubElement(actuator, 'position', name='shoulder_joint', joint='shoulder_joint', kp='100')
    ET.SubElement(actuator, 'position', name='elbow_joint', joint='elbow_joint', kp='100')
    
    # Velocity Actuators
    ET.SubElement(actuator, 'velocity', name='base_joint_vel', joint='base_joint', kv='10')
    ET.SubElement(actuator, 'velocity', name='shoulder_joint_vel', joint='shoulder_joint', kv='10')
    ET.SubElement(actuator, 'velocity', name='elbow_joint_vel', joint='elbow_joint', kv='10')
    
    # Effort (Torque) Actuators
    ET.SubElement(actuator, 'motor', name='base_joint_motor', joint='base_joint')
    ET.SubElement(actuator, 'motor', name='shoulder_joint_motor', joint='shoulder_joint')
    ET.SubElement(actuator, 'motor', name='elbow_joint_motor', joint='elbow_joint')
    
    tree.write(wrapper_xml_path, encoding='utf-8', xml_declaration=True)
    
    # 4. Use the updated URDF for robot_description in ROS
    robot_description = {'robot_description': mujoco_urdf_content.replace('$(find gim_control)', pkg_share)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Launch MuJoCo ROS2 Control Node
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_file]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager', '--inactive'],
    )

    arm_velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_velocity_controller', '-c', '/controller_manager', '--inactive'],
    )

    arm_effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_effort_controller', '-c', '/controller_manager'],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        mujoco_node,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, arm_velocity_spawner, arm_effort_spawner],
            )
        ),
    ])
