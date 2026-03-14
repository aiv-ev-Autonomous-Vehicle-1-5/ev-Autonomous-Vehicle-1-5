# serial_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Henes T870 serial bridge parameter file
    serial_bridge_parameter = DeclareLaunchArgument('serial_bridge_parameter',
        default_value=PathJoinSubstitution([
            FindPackageShare('t870_serial'), 'config', 'serial_bridge.param.yaml'
        ])
    )

    # Henes T870 Racing serial bridge
    serial_bridge = Node(
        package    = 't870_serial',
        executable = 'serial_bridge',
        name       = 'serial_bridge',
        output     = 'screen',
        parameters = [{LaunchConfiguration('serial_bridge_parameter')}]
    )

    # Auto mode + gear forward service call (after node startup)
    auto_mode_service_call = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/t870/mode_command',
                    't870_msgs/srv/ModeCommand',
                    '{manual_mode: false, emergency_stop: false, gear: 0}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        serial_bridge_parameter,
        serial_bridge,
        auto_mode_service_call
    ])