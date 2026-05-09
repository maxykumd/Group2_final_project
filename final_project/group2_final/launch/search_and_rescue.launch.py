# Name: Yossaphat Kulvatunyou & Nam Facchetti
# Module: search_and_rescue_launch.py - BT action node that broadcasts a static TF frame for a survivor.

"""Launch file for the Search and Rescue mission.

Brings up Nav2 localization + navigation, simulated service servers,
and the behavior tree entry point in a single command.

Usage:
    Terminal 1: ros2 launch rosbot_gazebo final_project_world.launch.py
    Terminal 2: ros2 launch group2_final search_and_rescue.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the search and rescue mission.

    Returns:
        A LaunchDescription containing all required nodes and includes.
    """
    pkg_share = get_package_share_directory('group2_final')

    # File paths 
    map_file = os.path.join(pkg_share, 'maps', 'final_project_map.yaml')
    nav2_params  = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    mission_params = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    # Launch arguments 
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz with the nav2 view.',
    )

    # Nav2 localization (AMCL) 
    # Tells the robot WHERE it is on the saved map
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ]),
        launch_arguments=[
            ('map',          map_file),
            ('use_sim_time', 'true'),
            ('params_file',  nav2_params),
            ('autostart',    'true'),
        ],
    )

    # Nav2 navigation stack 
    # Plans and executes paths to goal poses
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('autostart',    'true'),
            ('params_file',  nav2_params),
        ],
    )

    # Simulated service servers
    detect_server = Node(
        package='group2_final',
        executable='detect_survivor_server_exe',
        name='detect_survivor_server',
        output='screen',
        emulate_tty=True,
    )

    report_server = Node(
        package='group2_final',
        executable='report_survivor_server_exe',
        name='report_survivor_server',
        output='screen',
        emulate_tty=True,
    )

    # Behavior tree entry point 
    bt_node = Node(
        package='group2_final',
        executable='search_and_rescue_exe',
        name='search_and_rescue',
        output='screen',
        emulate_tty=True,
        parameters=[mission_params],
    )

    # Rviz 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        localization,    # AMCL first — robot must know where it is
        navigation,      # then nav stack
        detect_server,   # service servers
        report_server,
        rviz_node,
        bt_node,         # BT last — needs everything else to be up
    ])