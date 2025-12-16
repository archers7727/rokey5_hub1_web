# DB 마이그레이션 가이드 - Emergency Stop Recovery

## 문제 진단

긴급정지 버튼을 눌러도 `recovery_needed` 필드가 변경되지 않는 이유:
- `recovery_needed` 컬럼이 실제 Supabase DB에 추가되지 않았을 가능성

## 해결 방법

### 1️⃣ 현재 DB 스키마 확인

Supabase Dashboard → SQL Editor에서 실행:

```sql
-- check-robot-state-schema.sql 내용
SELECT
  column_name,
  data_type,
  column_default,
  is_nullable
FROM information_schema.columns
WHERE table_name = 'robot_state'
ORDER BY ordinal_position;

SELECT * FROM robot_state WHERE id = 'current';
```

**확인 사항:**
- `recovery_needed` 컬럼이 있는지 확인
- 없다면 → 다음 단계로 마이그레이션 실행 필요

---

### 2️⃣ DB 마이그레이션 실행

Supabase Dashboard → SQL Editor에서 `supabase-migration-complete.sql` 내용 실행:

```sql
-- 1. Add Doosan robot state columns (if not exists)
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS doosan_robot_state INTEGER DEFAULT 1,
ADD COLUMN IF NOT EXISTS desired_state TEXT,
ADD COLUMN IF NOT EXISTS command_timestamp TIMESTAMP WITH TIME ZONE;

-- 2. Add recovery column for emergency stop recovery flow
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS recovery_needed BOOLEAN DEFAULT false;

-- 3. Add comments
COMMENT ON COLUMN robot_state.doosan_robot_state IS
'Doosan robot state: 0=INITIALIZING, 1=STANDBY, 2=MOVING, 3=SAFE_OFF, 4=TEACHING, 5=SAFE_STOP, 6=EMERGENCY_STOP, 7=HOMMING, 8=RECOVERY, 9=SAFE_STOP2, 10=SAFE_OFF2, 11-14=RESERVED, 15=NOT_READY';

COMMENT ON COLUMN robot_state.desired_state IS
'Command from web UI: pause, resume, stop, emergency_stop, move_to_home';

COMMENT ON COLUMN robot_state.recovery_needed IS
'Indicates if robot needs recovery after emergency stop. Set to true when emergency_stop command is executed, set to false when robot reaches home position.';

-- 4. Set default values
UPDATE robot_state
SET
  doosan_robot_state = COALESCE(doosan_robot_state, 1),
  recovery_needed = COALESCE(recovery_needed, false)
WHERE id = 'current';
```

**예상 결과:**
```
Success. No rows returned
```

---

### 3️⃣ 마이그레이션 검증

다시 스키마 확인 쿼리 실행:

```sql
SELECT
  column_name,
  data_type,
  column_default
FROM information_schema.columns
WHERE table_name = 'robot_state'
ORDER BY ordinal_position;
```

**예상 출력:**
```
column_name          | data_type | column_default
---------------------|-----------|---------------
id                   | text      | 'current'
status               | text      | 'idle'
current_task_id      | uuid      | NULL
joint_states         | jsonb     | {...}
error_state          | text      | NULL
updated_at           | timestamp | NOW()
doosan_robot_state   | integer   | 1
desired_state        | text      | NULL
command_timestamp    | timestamp | NULL
recovery_needed      | boolean   | false        ← 이것이 있어야 함!
```

---

### 4️⃣ 긴급정지 테스트

마이그레이션 후:

1. **긴급정지 버튼 클릭**
   ```
   웹 → POST /api/robot/command { command: 'emergency_stop' }
   → desired_state = 'emergency_stop'
   ```

2. **DB 확인 (1-2초 후)**
   ```sql
   SELECT
     status,
     doosan_robot_state,
     recovery_needed,
     desired_state
   FROM robot_state
   WHERE id = 'current';
   ```

   **예상 결과:**
   ```
   status: 'error'
   doosan_robot_state: 6
   recovery_needed: true     ← 이것이 true가 되어야 함!
   desired_state: NULL       (ROS2가 처리 후 초기화)
   ```

3. **UI 확인**
   - 긴급정지 버튼이 → 🏠 "홈으로 이동" 버튼으로 변경되어야 함

---

## 추가 문제 해결

### ROS2 노드가 실행 중인지 확인

```bash
# ROS2 노드 로그 확인
ros2 node list  # robot_command_handler가 있는지 확인

# 노드 로그 보기
ros2 run robot_hub_nodes robot_command_handler_node.py
```

**확인 사항:**
- `Robot Command Handler Node started` 메시지가 보이는지
- `Monitoring desired_state field...` 메시지가 보이는지
- Supabase 연결 에러가 없는지

### Supabase 연결 확인

`robot_command_handler_node.py` 파일의 30-31번째 줄 확인:

```python
SUPABASE_URL = "https://your-project.supabase.co"  # 실제 URL로 변경했는지?
SUPABASE_ANON_KEY = "your-anon-key-here"  # 실제 키로 변경했는지?
```

**수정 필요 시:**
1. Supabase Dashboard → Settings → API
2. `URL`과 `anon/public` 키 복사
3. 코드에 붙여넣기

---

## 체크리스트

- [ ] Supabase SQL Editor에서 `supabase-migration-complete.sql` 실행
- [ ] `recovery_needed` 컬럼이 추가되었는지 확인
- [ ] 긴급정지 버튼 클릭 후 `recovery_needed=true`로 변경되는지 확인
- [ ] UI에서 버튼이 "홈으로 이동"으로 변경되는지 확인
- [ ] ROS2 노드가 실행 중인지 확인
- [ ] Supabase 연결 정보가 올바른지 확인

---

## 파일 목록

- `supabase-migration-complete.sql` - 통합 마이그레이션 스크립트
- `check-robot-state-schema.sql` - 스키마 확인용 쿼리
- `DB_MIGRATION_GUIDE.md` - 이 가이드 문서
