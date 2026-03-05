# carsa_gazebo_kcity.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    carsa_gazebo = IncludeLaunchDescription(

        # Launch file
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('erp42_gazebo'), 'launch', 'gazebo.launch.py'
            ])
        ),

        # Launch arguments
        launch_arguments = {

            # Vehicle description file
            'vehicle_description_file': PathJoinSubstitution([
                FindPackageShare('carsa_gazebo'), 'urdf', 't870.xacro'
            ]),

            # World file
            'world_file': PathJoinSubstitution([
                FindPackageShare('carsa_gazebo'), 'worlds', 'rubber-cone-1-rail.world'
            ]),

            # Spawn location
            'spawn_x'    : '0.0',
            'spawn_y'    : '0.0',
            'spawn_z'    : '0.0',
            'spawn_roll' : '0.0',
            'spawn_pitch': '0.0',
            'spawn_yaw'  : '0.0',

            # Rviz configureation file
            'rviz_config_file': PathJoinSubstitution([
                FindPackageShare('carsa_gazebo'), 'rviz', 't870.rviz'
            ]),

        }.items()
    )

    return LaunchDescription([
        carsa_gazebo
    ])