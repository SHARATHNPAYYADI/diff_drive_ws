from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get current package directory
    config_file = PathJoinSubstitution([
        FindPackageShare("diff_drive_bringup"),
        "config",
        "ekf.yaml"
    ])

    return LaunchDescription([
        Node(
            package='diff_drive_localization',  
            executable='scan_map_localizer',     
            name='ekf_fusion'
        ),
        Node(
            package='diff_drive_localization',  
            executable='ekf_fusion',     
            name='ekf_fusion',
            output='screen',
            parameters=[config_file]
        )
    ])
