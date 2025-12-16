#!/usr/bin/env python3
"""
Robot State Publisher Node - UPSERT 버전

UPDATE 대신 UPSERT를 사용하여 RLS 문제 해결
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from supabase import create_client, Client
from datetime import datetime


# Supabase 설정
SUPABASE_URL = "https://lxllllhkovbegtbcfnaw.supabase.co"
SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx4bGxsbGhrb3ZiZWd0YmNmbmF3Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjU4MDQ4MzksImV4cCI6MjA4MTM4MDgzOX0.MGL576AumztyxI7SEYUCxnPuo0euoWWFoEmTmJmfEEQ"


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # 로봇 이름 파라미터 선언 (기본값: dsr01)
        self.declare_parameter('robot_name', 'dsr01')
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Robot name: {self.robot_name}')

        # Supabase 클라이언트 초기화
        self.supabase: Client = create_client(SUPABASE_URL, SUPABASE_ANON_KEY)
        self.get_logger().info('Supabase client initialized')

        # 로봇 상태 데이터 저장
        self.joint_states = {
            'position': [0.0] * 6,
            'velocity': [0.0] * 6,
            'effort': [0.0] * 6
        }

        self.current_status = 'idle'

        # 데이터 변경 추적
        self.data_changed = False

        # 구독자 생성
        joint_states_topic = f'/{self.robot_name}/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )

        # Supabase 업데이트 타이머 (1Hz)
        self.update_timer = self.create_timer(1.0, self.update_supabase)

        # 로그 카운터
        self._update_count = 0

        self.get_logger().info('Robot State Publisher Node started')
        self.get_logger().info(f'Subscribing to: {joint_states_topic}')
        self.get_logger().info('Using UPSERT for Supabase updates')

    def joint_state_callback(self, msg: JointState):
        """관절 상태 콜백"""
        try:
            # 관절 상태 업데이트 - 명시적으로 float로 변환!
            if len(msg.position) >= 6:
                self.joint_states['position'] = [float(x) for x in msg.position[:6]]
                self.data_changed = True

            if len(msg.velocity) >= 6:
                self.joint_states['velocity'] = [float(x) for x in msg.velocity[:6]]
                self.data_changed = True

            if len(msg.effort) >= 6:
                self.joint_states['effort'] = [float(x) for x in msg.effort[:6]]
                self.data_changed = True

            # 로봇이 움직이고 있는지 확인
            new_status = 'running' if any(abs(v) > 0.01 for v in self.joint_states['velocity']) else 'idle'

            if new_status != self.current_status:
                self.current_status = new_status
                self.data_changed = True

        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {str(e)}')

    def update_supabase(self):
        """Supabase robot_state 테이블 업데이트 (UPSERT 사용)"""
        # 데이터 변경이 없으면 스킵
        if not self.data_changed:
            return

        try:
            # UPSERT할 데이터 준비 (id 포함 필수!)
            upsert_data = {
                'id': 'current',  # Primary key - UPSERT를 위해 반드시 필요
                'robot_name': self.robot_name,
                'status': self.current_status,
                'joint_states': self.joint_states,
                'updated_at': datetime.utcnow().isoformat()
            }

            # UPSERT 실행 (존재하면 UPDATE, 없으면 INSERT)
            result = self.supabase.table('robot_state')\
                .upsert(upsert_data)\
                .execute()

            # 데이터 변경 플래그 리셋
            self.data_changed = False

            # 로그 출력 (5초마다)
            self._update_count += 1
            if self._update_count % 5 == 0:
                self.get_logger().info(
                    f'✅ Robot state updated - Robot: {self.robot_name}, '
                    f'Status: {self.current_status}, '
                    f'Joint Pos: [{", ".join(f"{p:.3f}" for p in self.joint_states["position"][:3])}...]'
                )

        except Exception as e:
            self.get_logger().error(f'❌ Failed to update Supabase: {str(e)}')
            self.get_logger().error(f'Error type: {type(e).__name__}')

            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')

            # 에러 발생 시에도 플래그 리셋
            self.data_changed = False


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RobotStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
