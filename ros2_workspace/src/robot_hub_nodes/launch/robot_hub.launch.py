#!/usr/bin/env python3
"""
Robot Hub Launch File

이 launch 파일은 세 개의 노드를 동시에 실행합니다:
  1. robot_state_publisher: 로봇 상태를 Supabase에 업데이트
  2. task_monitor: Supabase 작업을 모니터링하고 ROS2 토픽으로 발행
  3. robot_command_handler: 웹 명령을 모니터링하고 로봇에 전달

사용법:
  ros2 launch robot_hub_nodes robot_hub.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher 노드
        Node(
            package='robot_hub_nodes',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # Task Monitor 노드
        Node(
            package='robot_hub_nodes',
            executable='task_monitor',
            name='task_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # Robot Command Handler 노드
        Node(
            package='robot_hub_nodes',
            executable='robot_command_handler',
            name='robot_command_handler',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
