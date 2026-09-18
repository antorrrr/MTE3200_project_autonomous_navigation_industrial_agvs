#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    performance_evaluator_node = Node(
        package='nav2_performance_evaluator',
        executable='evaluator',
        name='performance_evaluator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        performance_evaluator_node
    ])