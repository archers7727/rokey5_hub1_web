# Robot Hub Nodes

ROS2 패키지 - Supabase와 연동하여 로봇 상태를 관리하고 작업을 모니터링합니다.

## 노드 구성

### 1. robot_state_publisher
로봇의 관절 상태를 실시간으로 Supabase에 업데이트합니다.

**구독 토픽:**
- `/joint_states` (sensor_msgs/JointState): 로봇 관절 상태
- `/tcp_pose` (geometry_msgs/Pose): TCP 위치

**업데이트 주기:** 1Hz

### 2. task_monitor
Supabase의 tasks 테이블을 모니터링하여 새로운 작업을 ROS2 토픽으로 발행합니다.

**발행 토픽:**
- `/new_task` (std_msgs/String): 새로운 작업 정보 (JSON 형식)

**모니터링 방식:**
- Supabase Realtime (실시간 구독)
- 5초마다 pending 작업 확인 (폴링)

## 설치 방법

### 1. 의존성 설치

```bash
cd ~/rokey5_hub1_web/ros2_workspace/src/robot_hub_nodes
pip3 install -r requirements.txt
```

### 2. 환경 변수 설정

```bash
# .env 파일 생성
cp .env.example .env

# .env 파일 편집
nano .env
```

`.env` 파일에 Supabase 정보 입력:
```
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key-here
```

### 3. 패키지 빌드

```bash
cd ~/rokey5_hub1_web/ros2_workspace
colcon build --packages-select robot_hub_nodes
source install/setup.bash
```

## 실행 방법

### Launch 파일로 모든 노드 실행

```bash
ros2 launch robot_hub_nodes robot_hub.launch.py
```

### 개별 노드 실행

**Robot State Publisher 노드만 실행:**
```bash
ros2 run robot_hub_nodes robot_state_publisher
```

**Task Monitor 노드만 실행:**
```bash
ros2 run robot_hub_nodes task_monitor
```

## 토픽 확인

### 발행되는 토픽 확인

```bash
# 새로운 작업 토픽 확인
ros2 topic echo /new_task
```

### 구독 중인 토픽 확인

```bash
# 관절 상태 토픽
ros2 topic echo /joint_states

# TCP 위치 토픽
ros2 topic echo /tcp_pose
```

## 테스트

### 1. 로봇 상태 업데이트 테스트

```bash
# joint_states 토픽 발행 (테스트용)
ros2 topic pub /joint_states sensor_msgs/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  position: [0.0, 0.0, 1.57, 0.0, 1.57, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}" --once
```

Supabase에서 `robot_state` 테이블 확인 → 업데이트 확인

### 2. 작업 모니터링 테스트

Supabase에서 새로운 작업 추가:
```sql
INSERT INTO tasks (material_id, mode_id, parameters, status, priority)
VALUES ('onion', 'slicing', '{"size": "medium"}'::jsonb, 'pending', 1);
```

ROS2 토픽 확인:
```bash
ros2 topic echo /new_task
```

## 문제 해결

### Supabase 연결 실패
- `.env` 파일의 Supabase URL과 키 확인
- 네트워크 연결 확인
- Supabase 프로젝트 상태 확인

### Realtime 구독 실패
- Supabase에서 Realtime이 활성화되어 있는지 확인
- `tasks` 테이블이 Realtime publication에 추가되어 있는지 확인

### 토픽이 발행/구독되지 않음
- ROS2 노드가 실행 중인지 확인: `ros2 node list`
- 토픽 목록 확인: `ros2 topic list`
- 토픽 정보 확인: `ros2 topic info /new_task`

## 라이센스

MIT License
