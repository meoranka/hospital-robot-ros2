"""Start the room-name node; Gazebo, AMCL and Nav2 are started separately."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    """Configure and start the hospital room navigator."""
    default_rooms = str(Path(get_package_share_directory('navigation_pkg')) /
                        'config' / 'hospital_rooms.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rooms_file', default_value=default_rooms),
        DeclareLaunchArgument('localizer', default_value='amcl'),
        Node(
            package='navigation_pkg', executable='hospital_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'rooms_file': LaunchConfiguration('rooms_file'),
                'localizer': LaunchConfiguration('localizer'),
            }],
        ),
    ])
