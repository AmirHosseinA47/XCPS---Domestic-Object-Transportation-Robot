"""Starts Gazebo with the IAI kitchen and the PR2, the ROS 2 bridge, and RViz.

    ros2 launch kitchen_gazebo kitchen_sim.launch.py            # Gazebo window + RViz
    ros2 launch kitchen_gazebo kitchen_sim.launch.py gui:=false  # headless Gazebo
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The lab spawns the PR2 at (1, 2), but there its base overlaps the oven front, which the
# lab's kinematic world ignores and Gazebo's physics does not. The default is a free spot
# on the floor between the sink counter and the kitchen island.
ROBOT_START = {'x': '0.2', 'y': '0.6', 'yaw': '0.0'}


def generate_launch_description():
    share = get_package_share_directory('kitchen_gazebo')
    pr2_urdf_path = os.path.join(share, 'urdf', 'pr2_gazebo.urdf')
    kitchen_urdf_path = os.path.join(share, 'urdf', 'kitchen_gazebo.urdf')
    world = os.path.join(share, 'worlds', 'kitchen_world.sdf')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    with open(pr2_urdf_path) as f:
        pr2_description = f.read()
    with open(kitchen_urdf_path) as f:
        kitchen_description = f.read()

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # Lets Gazebo resolve package://pr2_description/... and package://iai_kitchen/... meshes.
    resource_paths = [os.path.dirname(get_package_share_directory(p))
                      for p in ('pr2_description', 'iai_kitchen')]
    resource_paths.append(os.path.join(share, 'models'))   # model://milk etc.

    # The objects stand on the counter (config/scene.yaml). They are spawned after the PR2 has
    # parked its arms: Harmonic welds each object to both hands when it appears, and
    # kitchen_skills.py lets go of them right away.
    with open(os.path.join(share, 'config', 'scene.yaml')) as f:
        scene = yaml.safe_load(f)
    spawn_objects = [
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'kitchen', '-name', name,
                        '-file', os.path.join(share, 'models', name, 'model.sdf'),
                        '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)])
        for name, (x, y, z, yaw) in scene['objects'].items()]

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r -v 2 {world}', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r -s -v 2 {world}', 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(gui))

    spawn_kitchen = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'kitchen', '-name', 'kitchen', '-file', kitchen_urdf_path])
    spawn_pr2 = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'kitchen', '-name', 'pr2', '-file', pr2_urdf_path,
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.0',
                   '-Y', LaunchConfiguration('yaw')])

    pr2_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': pr2_description, 'use_sim_time': True}])
    kitchen_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='kitchen', output='screen',
        parameters=[{'robot_description': kitchen_description, 'use_sim_time': True}])
    world_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--frame-id', 'world', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])
    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(rviz),
        arguments=['-d', os.path.join(share, 'rviz', 'kitchen.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        *[DeclareLaunchArgument(k, default_value=v, description=f'PR2 start {k}')
          for k, v in ROBOT_START.items()],
        *[AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in resource_paths],
        gazebo_with_gui,
        gazebo_headless,
        bridge,
        kitchen_state_publisher,
        pr2_state_publisher,
        world_to_odom,
        TimerAction(period=3.0, actions=[spawn_kitchen]),
        TimerAction(period=8.0, actions=[spawn_pr2]),
        TimerAction(period=10.0, actions=[rviz_node]),
        TimerAction(period=16.0, actions=spawn_objects),
    ])
