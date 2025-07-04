
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    teleop_dir = FindPackageShare('teleop')

    # Launch configurations
    joystick = LaunchConfiguration('joystick').perform(context)
    joy_device_L = LaunchConfiguration('joy_dev_L')
    joy_device_R = LaunchConfiguration('joy_dev_R')
    joy_vel = LaunchConfiguration('joy_vel')
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([teleop_dir, 'config', f'{joystick}.config.yaml']),
    )

    return [
        # Add Nodes
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_L',
            parameters=[
                {'device_id': joy_device_L,
                 'deadzone': 0.1,
                 'autorepeat_rate': 20.0}
            ],
            remappings=[
                ('joy', 'joy_L')
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_R',
            parameters=[
                {'device_id': joy_device_R,
                 'deadzone': 0.1,
                 'autorepeat_rate': 20.0}
            ],
            remappings=[
                ('joy', 'joy_R')
            ]
        ),
        Node(
            package='teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[params_file],
            remappings=[
                ('/cmd_vel', joy_vel),
            ],
        ),
        # Log Information
        LogInfo(msg=['Joystick Loaded: ', joystick]),
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            name='joystick',
            default_value='thrustmasters',
        ),
        DeclareLaunchArgument(
            name='joy_dev_L',
            default_value='0',
        ),
        DeclareLaunchArgument(
            name='joy_dev_R',
            default_value='1',
        ),
        DeclareLaunchArgument(
            name='joy_vel',
            default_value='cmd_vel'
        ),
        DeclareLaunchArgument(
            name='params_file',
            default_value='', #Defined in launch_setup due to requiring another launch argument
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
