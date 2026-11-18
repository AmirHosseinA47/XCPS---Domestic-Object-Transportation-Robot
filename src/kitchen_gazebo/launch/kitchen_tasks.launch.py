"""Simulation + Nav2 + the robot skills server: everything the behaviour tree needs.

    ros2 launch kitchen_gazebo kitchen_tasks.launch.py
    ros2 run kitchen_gazebo skills_demo.py --scenario set     # or: clean
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('kitchen_gazebo')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'kitchen_nav.launch.py')))
    skills = Node(package='kitchen_gazebo', executable='kitchen_skills.py', output='screen',
                  parameters=[{'use_sim_time': True}])
    return LaunchDescription([navigation, TimerAction(period=18.0, actions=[skills])])
