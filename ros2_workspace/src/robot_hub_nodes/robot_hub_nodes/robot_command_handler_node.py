#!/usr/bin/env python3
"""
Robot Command Handler Node

이 노드는 Supabase의 desired_state 필드를 Realtime으로 모니터링하여
웹에서 전송한 명령을 로봇에 전달합니다.

감시 필드:
  - desired_state (pause, resume, stop, emergency_stop)

발행 토픽:
  - /robot_command (std_msgs/String): 로봇 제어 명령

동작:
  1. Supabase Realtime으로 desired_state 감시
  2. 명령 감지 시 /robot_command 토픽으로 발행
  3. 명령 처리 후 desired_state를 NULL로 초기화
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from supabase import create_client, Client
import threading
from datetime import datetime

# ========================================
# Supabase 설정 (하드코딩)
# ========================================
SUPABASE_URL = "https://your-project.supabase.co"  # 여기에 실제 URL 입력
SUPABASE_ANON_KEY = "your-anon-key-here"  # 여기에 실제 키 입력


class RobotCommandHandler(Node):
    def __init__(self):
        super().__init__('robot_command_handler')

        # Supabase 클라이언트 초기화
        try:
            self.supabase: Client = create_client(SUPABASE_URL, SUPABASE_ANON_KEY)
            self.get_logger().info('Supabase client initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Supabase: {str(e)}')
            raise

        # ROS2 퍼블리셔 생성 - 로봇 제어 명령 발행
        self.command_publisher = self.create_publisher(
            String,
            '/robot_command',
            10
        )

        # 현재 처리 중인 명령 추적 (중복 방지)
        self.current_command = None
        self.processing_command = False

        # 초기 상태 로드
        self.load_initial_state()

        # Realtime 구독 시작 (별도 스레드)
        self.realtime_thread = threading.Thread(
            target=self.subscribe_to_realtime,
            daemon=True
        )
        self.realtime_thread.start()

        # 주기적으로 desired_state 확인 (fallback, 5초마다)
        self.check_timer = self.create_timer(5.0, self.check_desired_state)

        self.get_logger().info('Robot Command Handler Node started')
        self.get_logger().info('Monitoring desired_state field...')
        self.get_logger().info('Publishing commands to: /robot_command')

    def load_initial_state(self):
        """초기 로봇 상태 로드"""
        try:
            response = self.supabase.table('robot_state')\
                .select('*')\
                .eq('id', 'current')\
                .single()\
                .execute()

            if response.data:
                state = response.data
                if state.get('desired_state'):
                    self.get_logger().info(
                        f'Found pending command: {state["desired_state"]}'
                    )
                    self.process_command(state['desired_state'])
        except Exception as e:
            self.get_logger().error(f'Failed to load initial state: {str(e)}')

    def check_desired_state(self):
        """주기적으로 desired_state 확인 (fallback)"""
        if self.processing_command:
            return

        try:
            response = self.supabase.table('robot_state')\
                .select('desired_state')\
                .eq('id', 'current')\
                .single()\
                .execute()

            if response.data and response.data.get('desired_state'):
                desired_state = response.data['desired_state']
                if desired_state != self.current_command:
                    self.get_logger().info(
                        f'Polling detected new command: {desired_state}'
                    )
                    self.process_command(desired_state)
        except Exception as e:
            self.get_logger().error(f'Failed to check desired_state: {str(e)}')

    def subscribe_to_realtime(self):
        """Supabase Realtime 구독"""
        try:
            channel = self.supabase.channel('robot-command-channel')

            def on_update(payload):
                """robot_state 업데이트 감지"""
                try:
                    new_data = payload.get('new', {})
                    desired_state = new_data.get('desired_state')

                    if desired_state and desired_state != self.current_command:
                        self.get_logger().info(
                            f'Realtime detected new command: {desired_state}'
                        )
                        self.process_command(desired_state)
                except Exception as e:
                    self.get_logger().error(f'Error in Realtime callback: {str(e)}')

            # robot_state 테이블의 UPDATE 이벤트 구독
            channel.on_postgres_changes(
                event='UPDATE',
                schema='public',
                table='robot_state',
                callback=on_update
            )

            # 구독 시작
            channel.subscribe()
            self.get_logger().info('Realtime subscription active')

            # 연결 유지
            while rclpy.ok():
                import time
                time.sleep(1)

        except Exception as e:
            self.get_logger().error(f'Realtime subscription error: {str(e)}')

    def process_command(self, command: str):
        """명령 처리"""
        if self.processing_command:
            self.get_logger().warn(f'Already processing a command, skipping: {command}')
            return

        self.processing_command = True
        self.current_command = command

        try:
            # 유효한 명령인지 확인
            valid_commands = ['pause', 'resume', 'stop', 'emergency_stop']
            if command not in valid_commands:
                self.get_logger().warn(f'Invalid command: {command}')
                self.clear_command()
                return

            # ROS2 토픽으로 명령 발행
            self.publish_command(command)

            # 로봇 상태 업데이트 (시뮬레이션)
            self.update_robot_state(command)

            # 명령 처리 완료 - desired_state 초기화
            self.clear_command()

            self.get_logger().info(f'Command processed successfully: {command}')

        except Exception as e:
            self.get_logger().error(f'Failed to process command: {str(e)}')
        finally:
            self.processing_command = False

    def publish_command(self, command: str):
        """ROS2 토픽으로 명령 발행"""
        try:
            msg = String()
            msg.data = command
            self.command_publisher.publish(msg)

            self.get_logger().info(f'Published command to /robot_command: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish command: {str(e)}')

    def update_robot_state(self, command: str):
        """로봇 상태 업데이트 (명령에 따라)"""
        try:
            # 명령에 따른 상태 매핑
            state_mapping = {
                'pause': {
                    'status': 'paused',
                    'doosan_robot_state': 5  # STATE_SAFE_STOP
                },
                'resume': {
                    'status': 'running',
                    'doosan_robot_state': 2  # STATE_MOVING
                },
                'stop': {
                    'status': 'idle',
                    'doosan_robot_state': 1  # STATE_STANDBY
                },
                'emergency_stop': {
                    'status': 'error',
                    'doosan_robot_state': 6  # STATE_EMERGENCY_STOP
                }
            }

            if command in state_mapping:
                update_data = state_mapping[command]
                update_data['updated_at'] = datetime.utcnow().isoformat()

                # DB 업데이트
                self.supabase.table('robot_state')\
                    .update(update_data)\
                    .eq('id', 'current')\
                    .execute()

                self.get_logger().info(
                    f'Updated robot state: status={update_data["status"]}, '
                    f'doosan_robot_state={update_data["doosan_robot_state"]}'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to update robot state: {str(e)}')

    def clear_command(self):
        """처리 완료된 명령 초기화"""
        try:
            self.supabase.table('robot_state')\
                .update({'desired_state': None})\
                .eq('id', 'current')\
                .execute()

            self.get_logger().info('Cleared desired_state')
            self.current_command = None
        except Exception as e:
            self.get_logger().error(f'Failed to clear desired_state: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RobotCommandHandler()
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
