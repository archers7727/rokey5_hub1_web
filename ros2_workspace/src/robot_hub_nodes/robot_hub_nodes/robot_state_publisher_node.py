#!/usr/bin/env python3
"""
Robot State Publisher Node

이 노드는 로봇의 관절 상태를 ROS2 토픽에서 구독하여
Supabase 데이터베이스의 robot_state 테이블을 실시간으로 업데이트합니다.

구독 토픽:
  - /joint_states (sensor_msgs/JointState): 로봇 관절 상태
  - /tcp_pose (geometry_msgs/Pose): TCP(Tool Center Point) 위치

업데이트 주기: 1Hz (1초마다)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from supabase import create_client, Client
import os
from dotenv import load_dotenv
from datetime import datetime

# 환경 변수 로드
load_dotenv()


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Supabase 클라이언트 초기화
        supabase_url = os.getenv('NEXT_PUBLIC_SUPABASE_URL')
        supabase_key = os.getenv('NEXT_PUBLIC_SUPABASE_ANON_KEY')

        if not supabase_url or not supabase_key:
            self.get_logger().error('Supabase credentials not found in environment variables!')
            raise ValueError('Missing Supabase credentials')

        self.supabase: Client = create_client(supabase_url, supabase_key)
        self.get_logger().info('Supabase client initialized')

        # 로봇 상태 데이터 저장
        self.joint_states = {
            'position': [0.0] * 6,
            'velocity': [0.0] * 6,
            'effort': [0.0] * 6
        }

        self.tcp_position = {
            'x': 500.0,
            'y': 0.0,
            'z': 300.0,
            'rx': 0.0,
            'ry': 0.0,
            'rz': 0.0
        }

        self.current_status = 'idle'  # idle, running, paused, error

        # 구독자 생성
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.tcp_pose_sub = self.create_subscription(
            Pose,
            '/tcp_pose',
            self.tcp_pose_callback,
            10
        )

        # Supabase 업데이트 타이머 (1Hz)
        self.update_timer = self.create_timer(1.0, self.update_supabase)

        self.get_logger().info('Robot State Publisher Node started')
        self.get_logger().info('Subscribing to: /joint_states, /tcp_pose')
        self.get_logger().info('Updating Supabase at 1Hz')

    def joint_state_callback(self, msg: JointState):
        """관절 상태 콜백"""
        try:
            # 관절 상태 업데이트
            if len(msg.position) >= 6:
                self.joint_states['position'] = list(msg.position[:6])

            if len(msg.velocity) >= 6:
                self.joint_states['velocity'] = list(msg.velocity[:6])

            if len(msg.effort) >= 6:
                self.joint_states['effort'] = list(msg.effort[:6])

            # 로봇이 움직이고 있는지 확인 (속도 기반)
            if any(abs(v) > 0.01 for v in self.joint_states['velocity']):
                self.current_status = 'running'
            else:
                self.current_status = 'idle'

        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {str(e)}')

    def tcp_pose_callback(self, msg: Pose):
        """TCP 위치 콜백"""
        try:
            self.tcp_position = {
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z,
                'rx': msg.orientation.x,
                'ry': msg.orientation.y,
                'rz': msg.orientation.z
            }
        except Exception as e:
            self.get_logger().error(f'Error in tcp_pose_callback: {str(e)}')

    def update_supabase(self):
        """Supabase robot_state 테이블 업데이트"""
        try:
            # 업데이트할 데이터 준비
            update_data = {
                'status': self.current_status,
                'joint_states': self.joint_states,
                'tcp_position': self.tcp_position,
                'updated_at': datetime.utcnow().isoformat()
            }

            # Supabase 업데이트
            result = self.supabase.table('robot_state').update(update_data).eq('id', 'current').execute()

            # 로그 출력 (5초마다 한 번씩만)
            if not hasattr(self, '_update_count'):
                self._update_count = 0

            self._update_count += 1
            if self._update_count % 5 == 0:
                self.get_logger().info(
                    f'Robot state updated - Status: {self.current_status}, '
                    f'Joint Pos: [{", ".join(f"{p:.2f}" for p in self.joint_states["position"][:3])}...]'
                )

        except Exception as e:
            self.get_logger().error(f'Failed to update Supabase: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RobotStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
