#!/usr/bin/env python3
"""
Test Task Subscriber

이 노드는 /new_task 토픽을 구독하여 새로운 작업을 받아
처리하는 예제 노드입니다.

실제 로봇 제어 노드에서 이 패턴을 참고하여
작업을 받아 실행할 수 있습니다.

구독 토픽:
  - /new_task (std_msgs/String): 새로운 작업 정보 (JSON 형식)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class TestTaskSubscriber(Node):
    def __init__(self):
        super().__init__('test_task_subscriber')

        # /new_task 토픽 구독
        self.task_subscription = self.create_subscription(
            String,
            '/new_task',
            self.task_callback,
            10
        )

        self.get_logger().info('Test Task Subscriber started')
        self.get_logger().info('Waiting for new tasks on /new_task...')

    def task_callback(self, msg: String):
        """새로운 작업 콜백"""
        try:
            # JSON 파싱
            task_data = json.loads(msg.data)

            self.get_logger().info('=' * 60)
            self.get_logger().info('📋 New Task Received!')
            self.get_logger().info(f'  Task ID: {task_data.get("id")}')
            self.get_logger().info(f'  Material: {task_data.get("material_id")}')
            self.get_logger().info(f'  Mode: {task_data.get("mode_id")}')
            self.get_logger().info(f'  Parameters: {task_data.get("parameters")}')
            self.get_logger().info(f'  Priority: {task_data.get("priority")}')
            self.get_logger().info(f'  Created: {task_data.get("created_at")}')
            self.get_logger().info('=' * 60)

            # 여기서 실제 작업 처리 로직을 구현합니다
            # 예: 로봇 제어 명령 전송, 경로 계획 등
            self.process_task(task_data)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in task_callback: {str(e)}')

    def process_task(self, task_data):
        """작업 처리 (예제)"""
        material = task_data.get('material_id')
        mode = task_data.get('mode_id')

        self.get_logger().info(f'Processing: {material} with {mode} mode...')

        # 실제 로봇 제어 코드는 여기에 구현
        # 예시:
        # - 재료에 따라 그리퍼 설정
        # - 모드에 따라 동작 경로 생성
        # - 로봇 제어 명령 전송

        self.get_logger().info('Task processing complete!')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TestTaskSubscriber()
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
