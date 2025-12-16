# ROS2 노드 Supabase 연결 가이드

## 문제 상황

긴급정지 버튼을 눌렀을 때:
- ✅ `desired_state`는 'emergency_stop'으로 변경됨
- ❌ `recovery_needed`는 여전히 false
- ❌ `desired_state`가 NULL로 초기화되지 않음

**원인**: ROS2 노드가 Supabase에 연결되지 않아서 DB 변경을 감지하지 못함

---

## 해결 방법

### 1️⃣ frontend-next/.env.local 파일 확인

프로젝트 루트에서:

```bash
cat frontend-next/.env.local
```

**필수 내용:**
```env
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**파일이 없다면** Supabase Dashboard에서 키 가져오기:
1. Supabase Dashboard → Settings → API
2. `Project URL` 복사
3. `Project API keys` → `anon` `public` 키 복사
4. `frontend-next/.env.local` 파일 생성 후 붙여넣기

---

### 2️⃣ python-dotenv 설치 (선택사항, 권장)

```bash
# ROS2 workspace에서
cd ros2_workspace
pip3 install python-dotenv
```

**설치하면**: 자동으로 `.env.local` 파일에서 환경변수를 읽음
**설치 안하면**: 수동으로 환경변수 export 필요

---

### 3️⃣ ROS2 노드 실행

#### 방법 A: python-dotenv 설치했다면

```bash
cd ros2_workspace
source install/setup.bash
ros2 run robot_hub_nodes robot_command_handler_node
```

**예상 출력:**
```
✅ Loaded .env.local from: /home/user/rokey5_hub1_web/frontend-next/.env.local
✅ Supabase URL: https://xxxxx.supabase.co
✅ Supabase Key: eyJhbGciOiJIUzI1NiI...xyz
[INFO] Robot Command Handler Node started
[INFO] Monitoring desired_state field...
```

#### 방법 B: python-dotenv 없다면

```bash
# 환경변수 수동 설정
export NEXT_PUBLIC_SUPABASE_URL="https://your-project.supabase.co"
export NEXT_PUBLIC_SUPABASE_ANON_KEY="your-anon-key-here"

# ROS2 노드 실행
cd ros2_workspace
source install/setup.bash
ros2 run robot_hub_nodes robot_command_handler_node
```

---

### 4️⃣ 테스트

1. **웹페이지에서 긴급정지 버튼 클릭**

2. **ROS2 노드 로그 확인**:
   ```
   [INFO] Realtime detected new command: emergency_stop
   [INFO] Published command to /robot_command: emergency_stop
   [INFO] Updated robot state: status=error, doosan_robot_state=6
   [INFO] Cleared desired_state
   ```

3. **DB 확인** (1-2초 후):
   ```sql
   SELECT status, doosan_robot_state, recovery_needed, desired_state
   FROM robot_state
   WHERE id = 'current';
   ```

   **예상 결과:**
   ```
   status: 'error'
   doosan_robot_state: 6
   recovery_needed: true      ← 이제 true!
   desired_state: NULL        ← 초기화됨!
   ```

4. **웹페이지 확인**:
   - 긴급정지 버튼이 → 🏠 "홈으로 이동" 버튼으로 변경

---

## 트러블슈팅

### ROS2 노드가 시작 안됨

**에러 메시지:**
```
❌ Supabase credentials not found!
```

**해결:**
- `frontend-next/.env.local` 파일 존재 확인
- 파일에 `NEXT_PUBLIC_SUPABASE_URL`, `NEXT_PUBLIC_SUPABASE_ANON_KEY` 있는지 확인

---

### desired_state는 바뀌는데 recovery_needed는 안 바뀜

**원인:** `recovery_needed` 컬럼이 DB에 없음

**해결:** Supabase SQL Editor에서 실행
```sql
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS recovery_needed BOOLEAN DEFAULT false;
```

---

### ROS2 노드가 Realtime 이벤트를 받지 못함

**확인:**
```bash
# 다른 터미널에서
ros2 topic list
# /robot_command가 보여야 함

ros2 topic echo /robot_command
# 긴급정지 버튼 누르면 메시지가 보여야 함
```

**해결:** ROS2 노드 재시작

---

## 정리

1. ✅ `frontend-next/.env.local` 파일 확인/생성
2. ✅ `pip3 install python-dotenv` 설치
3. ✅ ROS2 노드 실행
4. ✅ 긴급정지 테스트
5. ✅ `recovery_needed`가 true로 변경되는지 확인
