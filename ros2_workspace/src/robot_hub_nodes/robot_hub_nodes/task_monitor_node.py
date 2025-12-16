#!/usr/bin/env python3
"""
Task Monitor Node

이 노드는 Supabase의 tasks 테이블을 모니터링하여
pending 상태의 새로운 작업이 추가되면 ROS2 토픽으로 발행합니다.

발행 토픽:
  - /new_task (std_msgs/String): 새로운 작업 정보 (JSON 형식)

Supabase Realtime을 사용하여 실시간으로 작업을 모니터링합니다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from supabase import create_client, Client
import os
from dotenv import load_dotenv
import json
import threading

# 환경 변수 로드
load_dotenv()


class TaskMonitor(Node):
    def __init__(self):
        super().__init__('task_monitor')

        # Supabase 클라이언트 초기화
        supabase_url = os.getenv('NEXT_PUBLIC_SUPABASE_URL')
        supabase_key = os.getenv('NEXT_PUBLIC_SUPABASE_ANON_KEY')

        if not supabase_url or not supabase_key:
            self.get_logger().error('Supabase credentials not found in environment variables!')
            raise ValueError('Missing Supabase credentials')

        self.supabase: Client = create_client(supabase_url, supabase_key)
        self.get_logger().info('Supabase client initialized')

        # ROS2 퍼블리셔 생성
        self.task_publisher = self.create_subscription(
            String,
            '/new_task',
            10
        )
        # 실제 퍼블리셔 (구독이 아니라 발행)
        self.task_publisher = self.create_publisher(String, '/new_task', 10)

        self.get_logger().info('Task Monitor Node started')
        self.get_logger().info('Monitoring Supabase tasks table...')
        self.get_logger().info('Publishing to: /new_task')

        # 기존 pending 작업 확인 타이머 (5초마다)
        self.check_timer = self.create_timer(5.0, self.check_pending_tasks)

        # Realtime 구독 시작 (별도 스레드)
        self.realtime_thread = threading.Thread(target=self.subscribe_to_realtime, daemon=True)
        self.realtime_thread.start()

        # 처리된 작업 ID 추적 (중복 방지)
        self.processed_task_ids = set()

    def check_pending_tasks(self):
        """기존 pending 상태 작업 확인"""
        try:
            # pending 상태인 작업 조회
            response = self.supabase.table('tasks')\
                .select('*')\
                .eq('status', 'pending')\
                .order('created_at', desc=False)\
                .execute()

            if response.data:
                for task in response.data:
                    task_id = task['id']

                    # 이미 처리한 작업은 건너뛰기
                    if task_id in self.processed_task_ids:
                        continue

                    # 새로운 작업 처리
                    self.publish_task(task)
                    self.processed_task_ids.add(task_id)

        except Exception as e:
            self.get_logger().error(f'Error checking pending tasks: {str(e)}')

    def subscribe_to_realtime(self):
        """Supabase Realtime 구독"""
        try:
            # Realtime 채널 생성 및 구독
            channel = self.supabase.channel('task-monitor-channel')

            # INSERT 이벤트 리스너
            def on_insert(payload):
                self.get_logger().info('New task detected via Realtime!')
                task_data = payload.get('new', {})

                if task_data.get('status') == 'pending':
                    task_id = task_data.get('id')

                    # 중복 확인
                    if task_id not in self.processed_task_ids:
                        self.publish_task(task_data)
                        self.processed_task_ids.add(task_id)

            # UPDATE 이벤트 리스너 (상태 변경 모니터링)
            def on_update(payload):
                task_data = payload.get('new', {})
                old_data = payload.get('old', {})

                # pending으로 변경된 경우
                if task_data.get('status') == 'pending' and old_data.get('status') != 'pending':
                    self.get_logger().info('Task status changed to pending!')
                    task_id = task_data.get('id')

                    if task_id not in self.processed_task_ids:
                        self.publish_task(task_data)
                        self.processed_task_ids.add(task_id)

            # Realtime 구독 설정
            channel.on_postgres_changes(
                event='INSERT',
                schema='public',
                table='tasks',
                callback=on_insert
            )

            channel.on_postgres_changes(
                event='UPDATE',
                schema='public',
                table='tasks',
                callback=on_update
            )

            # 구독 시작
            channel.subscribe()

            self.get_logger().info('Realtime subscription active')

            # 연결 유지 (이 스레드는 계속 실행됨)
            while rclpy.ok():
                import time
                time.sleep(1)

        except Exception as e:
            self.get_logger().error(f'Realtime subscription error: {str(e)}')

    def publish_task(self, task_data):
        """새로운 작업을 ROS2 토픽으로 발행"""
        try:
            # 작업 정보를 JSON 문자열로 변환
            task_json = json.dumps({
                'id': task_data.get('id'),
                'material_id': task_data.get('material_id'),
                'mode_id': task_data.get('mode_id'),
                'parameters': task_data.get('parameters', {}),
                'priority': task_data.get('priority', 1),
                'created_at': task_data.get('created_at')
            })

            # ROS2 메시지 생성 및 발행
            msg = String()
            msg.data = task_json
            self.task_publisher.publish(msg)

            self.get_logger().info(
                f'Published new task: {task_data.get("id")} '
                f'(Material: {task_data.get("material_id")}, '
                f'Mode: {task_data.get("mode_id")})'
            )

            # 작업 상태를 'queued'로 업데이트 (선택사항)
            # self.supabase.table('tasks').update({'status': 'queued'}).eq('id', task_data.get('id')).execute()

        except Exception as e:
            self.get_logger().error(f'Error publishing task: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TaskMonitor()
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
