from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    urdf_path = PathJoinSubstitution([
        FindPackageShare("diff_drive_robot_description"),
        "urdf",
        "my_robot.urdf.xacro"
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("diff_drive_bringup"),
        "rviz",
        "urdf_config.rviz"
    ])
    map_path = PathJoinSubstitution([
        FindPackageShare("diff_drive_bringup"),
        "maps",
        "my_new_map.yaml"
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare("diff_drive_bringup"),
        "worlds",
        "construction_site.world"
    ])

    # Robot State Publisher (loads URDF with xacro)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_path])
        }]
    )

    # Gazebo (include its launch)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ]),
        launch_arguments={"world": world_path}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot"],
        output="screen"
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    # Map Server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_path}]
    )

    # Static transform map → odom
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    #map server
    map_server_bringup = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "nav2_util", "lifecycle_bringup", "map_server"],
                shell=False,
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        rviz_node,
        map_server_node,
        static_tf,
        map_server_bringup
    ])
