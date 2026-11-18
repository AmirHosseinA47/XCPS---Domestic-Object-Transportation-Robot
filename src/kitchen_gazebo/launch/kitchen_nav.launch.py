"""Kitchen simulation plus Nav2: map, path planning and driving for the PR2.

    ros2 launch kitchen_gazebo kitchen_nav.launch.py             # Gazebo window + RViz
    ros2 launch kitchen_gazebo kitchen_nav.launch.py gui:=false   # headless Gazebo, RViz only

In RViz, "2D Goal Pose" sends the PR2 to a clicked pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('kitchen_gazebo')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                               'navigation_launch.py')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    kitchen_map = os.path.join(share, 'maps', 'kitchen.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'kitchen_sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false'}.items())

    # The simulator's odometry is exact, so the map and odom frames coincide (no localization).
    map_to_world = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'world'],
        parameters=[{'use_sim_time': True}])
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'yaml_filename': kitchen_map, 'use_sim_time': True}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}])
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={'params_file': params, 'use_sim_time': 'true',
                          'autostart': 'true'}.items())
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        simulation,
        map_to_world,
        map_server,
        map_lifecycle,
        TimerAction(period=12.0, actions=[navigation]),  # after the PR2 has been spawned
        TimerAction(period=12.0, actions=[rviz]),
    ])
