# 🔄 Vercel + Supabase 마이그레이션 가이드

## 📋 목차
1. [현재 아키텍처](#현재-아키텍처)
2. [새로운 아키텍처](#새로운-아키텍처)
3. [마이그레이션 이유](#마이그레이션-이유)
4. [마이그레이션 단계](#마이그레이션-단계)
5. [데이터베이스 스키마](#데이터베이스-스키마)
6. [API 변경사항](#api-변경사항)
7. [배포 설정](#배포-설정)

---

## 🏗️ 현재 아키텍처

```
Frontend (React + Vite)
    ↓ HTTP/WebSocket
Backend (FastAPI + Python)
    ↓
Firebase Firestore (NoSQL DB)
    ↓
ROS2 Robot Control (예정)
```

**문제점:**
- Python 의존성 설치 에러 (Windows 환경, Rust 컴파일러 필요)
- FastAPI 백엔드 별도 관리 필요
- Firebase Admin SDK 설정 복잡
- Service Account 키 관리 어려움
- 로컬 개발 환경 구축 복잡

---

## 🚀 새로운 아키텍처

```
Frontend (React + Vite)
    ↓
Vercel Serverless Functions (Next.js API Routes 또는 Edge Functions)
    ↓
Supabase
    ├─ PostgreSQL (관계형 DB)
    ├─ Realtime (WebSocket 대체)
    ├─ Auth (사용자 인증 - 선택사항)
    └─ Storage (파일 저장 - 선택사항)
    ↓
ROS2 Robot Control (Supabase Realtime 구독)
```

**장점:**
- ✅ Python 의존성 문제 해결 (JavaScript/TypeScript만 사용)
- ✅ 프론트엔드 + 백엔드 통합 배포 (Vercel)
- ✅ 실시간 데이터 동기화 내장 (Supabase Realtime)
- ✅ 로컬 개발 환경 간단 (npm install만)
- ✅ 환경 변수 관리 간편 (Vercel + Supabase 대시보드)
- ✅ 무료 티어로 개발 가능

---

## 🎯 마이그레이션 단계

### Phase 1: 프로젝트 구조 변경 (React → Next.js)

#### 1.1 Next.js 프로젝트 생성

```bash
# 프로젝트 루트에서
npx create-next-app@latest frontend-next --typescript --tailwind --app --no-src-dir

cd frontend-next
npm install @supabase/supabase-js
npm install @supabase/ssr
```

#### 1.2 기존 React 컴포넌트 이동

```bash
# 기존 frontend/src 파일들을 frontend-next/app으로 이동
frontend/src/pages/          → frontend-next/app/(routes)/
frontend/src/components/     → frontend-next/components/
frontend/src/hooks/          → frontend-next/hooks/
frontend/shared/types/       → frontend-next/types/
```

**주요 변경사항:**
- `React Router` → `Next.js App Router`
- `useNavigate()` → `useRouter()` from `next/navigation`
- `<Link to="/path">` → `<Link href="/path">`

---

### Phase 2: Supabase 설정

#### 2.1 Supabase 프로젝트 생성

1. https://supabase.com 접속
2. "New Project" 클릭
3. 프로젝트 이름: `rokey-robot-hub`
4. Database Password 설정 (저장 필수!)
5. Region: `Northeast Asia (Seoul)` 선택

#### 2.2 환경 변수 설정

**`.env.local` (로컬 개발용)**
```env
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
SUPABASE_SERVICE_ROLE_KEY=your-service-role-key
```

**Vercel 배포 환경 변수 설정:**
```bash
# Vercel 대시보드에서 설정
# Settings → Environment Variables
NEXT_PUBLIC_SUPABASE_URL
NEXT_PUBLIC_SUPABASE_ANON_KEY
SUPABASE_SERVICE_ROLE_KEY
```

#### 2.3 Supabase 클라이언트 생성

**`lib/supabase/client.ts` (클라이언트 사이드)**
```typescript
import { createBrowserClient } from '@supabase/ssr'

export function createClient() {
  return createBrowserClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
  )
}
```

**`lib/supabase/server.ts` (서버 사이드)**
```typescript
import { createServerClient, type CookieOptions } from '@supabase/ssr'
import { cookies } from 'next/headers'

export function createClient() {
  const cookieStore = cookies()

  return createServerClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!,
    {
      cookies: {
        get(name: string) {
          return cookieStore.get(name)?.value
        },
      },
    }
  )
}
```

---

### Phase 3: 데이터베이스 스키마

#### 3.1 Supabase SQL Editor에서 실행

```sql
-- ========================================
-- 1. Materials (재료) 테이블
-- ========================================
CREATE TABLE materials (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  emoji TEXT NOT NULL,
  description TEXT,
  category TEXT NOT NULL,
  sizes JSONB NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- 초기 데이터 삽입
INSERT INTO materials (id, name, emoji, description, category, sizes) VALUES
('onion', '양파', '🧅', '신선한 양파로 다양한 요리를', 'vegetable', '{"small": 6, "medium": 8, "large": 10}'),
('potato', '감자', '🥔', '고소한 감자로 튀김 요리를', 'vegetable', '{"small": 5, "medium": 7, "large": 9}');

-- ========================================
-- 2. Modes (손질 모드) 테이블
-- ========================================
CREATE TABLE modes (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  icon TEXT NOT NULL,
  description TEXT,
  compatible_materials TEXT[] NOT NULL,
  duration INTEGER NOT NULL,
  difficulty TEXT NOT NULL,
  steps JSONB NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- 초기 데이터 삽입
INSERT INTO modes (id, name, icon, description, compatible_materials, duration, difficulty, steps) VALUES
('frying', '튀김', '🍤', '바삭하게 튀겨드립니다', ARRAY['onion', 'potato'], 180, 'medium',
  '[
    {"id": 1, "name": "재료 준비", "duration": 30, "description": "재료를 깨끗이 씻고 준비합니다"},
    {"id": 2, "name": "손질", "duration": 60, "description": "적당한 크기로 자릅니다"},
    {"id": 3, "name": "튀김", "duration": 90, "description": "180도 기름에 튀깁니다"}
  ]'::jsonb),
('slicing', '썰기', '🔪', '얇고 균일하게 썰어드립니다', ARRAY['onion', 'potato'], 120, 'easy',
  '[
    {"id": 1, "name": "재료 준비", "duration": 20, "description": "재료를 깨끗이 씻습니다"},
    {"id": 2, "name": "썰기", "duration": 100, "description": "얇고 균일하게 썰어냅니다"}
  ]'::jsonb);

-- ========================================
-- 3. Tasks (작업 큐) 테이블
-- ========================================
CREATE TABLE tasks (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  material_id TEXT NOT NULL REFERENCES materials(id),
  mode_id TEXT NOT NULL REFERENCES modes(id),
  parameters JSONB NOT NULL,
  status TEXT NOT NULL DEFAULT 'pending' CHECK (status IN ('pending', 'running', 'paused', 'completed', 'failed')),
  priority INTEGER DEFAULT 1,
  progress INTEGER DEFAULT 0 CHECK (progress >= 0 AND progress <= 100),
  current_step INTEGER,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  started_at TIMESTAMP WITH TIME ZONE,
  completed_at TIMESTAMP WITH TIME ZONE,
  estimated_time INTEGER,
  error_message TEXT
);

-- 인덱스 생성 (성능 최적화)
CREATE INDEX idx_tasks_status ON tasks(status);
CREATE INDEX idx_tasks_created_at ON tasks(created_at DESC);

-- ========================================
-- 4. Robot State (로봇 상태) 테이블
-- ========================================
CREATE TABLE robot_state (
  id TEXT PRIMARY KEY DEFAULT 'current',
  status TEXT NOT NULL DEFAULT 'idle' CHECK (status IN ('idle', 'running', 'paused', 'error')),
  current_task_id UUID REFERENCES tasks(id),
  joint_states JSONB NOT NULL DEFAULT '{"position": [0,0,90,0,90,0], "velocity": [0,0,0,0,0,0], "effort": [0,0,0,0,0,0]}',
  tcp_position JSONB NOT NULL DEFAULT '{"x": 500.0, "y": 0.0, "z": 300.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}',
  error_state TEXT,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- 초기 로봇 상태 삽입
INSERT INTO robot_state (id) VALUES ('current');

-- ========================================
-- 5. Jobs (완료된 작업 히스토리) 테이블
-- ========================================
CREATE TABLE jobs (
  id UUID PRIMARY KEY,
  material_id TEXT NOT NULL,
  mode_id TEXT NOT NULL,
  parameters JSONB NOT NULL,
  status TEXT NOT NULL,
  progress INTEGER NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  started_at TIMESTAMP WITH TIME ZONE,
  completed_at TIMESTAMP WITH TIME ZONE NOT NULL,
  estimated_time INTEGER,
  actual_time INTEGER,
  error_message TEXT
);

CREATE INDEX idx_jobs_completed_at ON jobs(completed_at DESC);

-- ========================================
-- 6. Realtime 활성화
-- ========================================
-- Supabase Realtime 구독을 위한 설정
ALTER PUBLICATION supabase_realtime ADD TABLE tasks;
ALTER PUBLICATION supabase_realtime ADD TABLE robot_state;
```

#### 3.2 Row Level Security (RLS) 설정

```sql
-- RLS 활성화 (프로덕션에서 필수)
ALTER TABLE materials ENABLE ROW LEVEL SECURITY;
ALTER TABLE modes ENABLE ROW LEVEL SECURITY;
ALTER TABLE tasks ENABLE ROW LEVEL SECURITY;
ALTER TABLE robot_state ENABLE ROW LEVEL SECURITY;
ALTER TABLE jobs ENABLE ROW LEVEL SECURITY;

-- 모든 사용자에게 읽기 권한 (개발용)
CREATE POLICY "Allow public read" ON materials FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON modes FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON tasks FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON robot_state FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON jobs FOR SELECT USING (true);

-- 모든 사용자에게 쓰기 권한 (개발용 - 나중에 인증 추가 필요)
CREATE POLICY "Allow public insert" ON tasks FOR INSERT WITH CHECK (true);
CREATE POLICY "Allow public update" ON tasks FOR UPDATE USING (true);
CREATE POLICY "Allow public update" ON robot_state FOR UPDATE USING (true);
```

---

### Phase 4: API Routes 변경

#### 4.1 Materials API

**`app/api/materials/route.ts`**
```typescript
import { NextResponse } from 'next/server'
import { createClient } from '@/lib/supabase/server'

// GET /api/materials
export async function GET() {
  const supabase = createClient()

  const { data: materials, error } = await supabase
    .from('materials')
    .select('*')
    .order('name')

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: materials })
}
```

#### 4.2 Modes API

**`app/api/modes/route.ts`**
```typescript
import { NextResponse } from 'next/server'
import { createClient } from '@/lib/supabase/server'

// GET /api/modes?material=onion
export async function GET(request: Request) {
  const { searchParams } = new URL(request.url)
  const material = searchParams.get('material')

  const supabase = createClient()

  let query = supabase
    .from('modes')
    .select('*')
    .order('name')

  // 재료별 필터링
  if (material) {
    query = query.contains('compatible_materials', [material])
  }

  const { data: modes, error } = await query

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: modes })
}
```

#### 4.3 Jobs API

**`app/api/jobs/route.ts`**
```typescript
import { NextResponse } from 'next/server'
import { createClient } from '@/lib/supabase/server'

// GET /api/jobs
export async function GET() {
  const supabase = createClient()

  const { data: jobs, error } = await supabase
    .from('jobs')
    .select('*')
    .order('completed_at', { ascending: false })
    .limit(50)

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: jobs })
}

// POST /api/jobs
export async function POST(request: Request) {
  const body = await request.json()
  const supabase = createClient()

  // tasks 테이블에 새 작업 추가
  const { data: task, error } = await supabase
    .from('tasks')
    .insert({
      material_id: body.material,
      mode_id: body.mode,
      parameters: body.parameters,
      status: 'pending',
      priority: 1,
      progress: 0,
      estimated_time: body.estimatedTime || 0,
    })
    .select()
    .single()

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({
    success: true,
    data: task,
    message: 'Task queued successfully'
  })
}
```

#### 4.4 Robot State API

**`app/api/robot/state/route.ts`**
```typescript
import { NextResponse } from 'next/server'
import { createClient } from '@/lib/supabase/server'

// GET /api/robot/state
export async function GET() {
  const supabase = createClient()

  const { data: state, error } = await supabase
    .from('robot_state')
    .select('*')
    .eq('id', 'current')
    .single()

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: state })
}
```

---

### Phase 5: Realtime 구독 (WebSocket 대체)

#### 5.1 Tasks Realtime Hook

**`hooks/useTasksRealtime.ts`**
```typescript
'use client'

import { useEffect, useState } from 'react'
import { createClient } from '@/lib/supabase/client'
import type { RealtimeChannel } from '@supabase/supabase-js'

export function useTasksRealtime() {
  const [tasks, setTasks] = useState<any[]>([])
  const supabase = createClient()

  useEffect(() => {
    // 초기 데이터 로드
    supabase
      .from('tasks')
      .select('*')
      .order('created_at', { ascending: false })
      .then(({ data }) => {
        if (data) setTasks(data)
      })

    // Realtime 구독
    const channel: RealtimeChannel = supabase
      .channel('tasks-channel')
      .on(
        'postgres_changes',
        {
          event: '*', // INSERT, UPDATE, DELETE 모두 구독
          schema: 'public',
          table: 'tasks'
        },
        (payload) => {
          console.log('Task changed:', payload)

          if (payload.eventType === 'INSERT') {
            setTasks(prev => [payload.new, ...prev])
          } else if (payload.eventType === 'UPDATE') {
            setTasks(prev =>
              prev.map(task =>
                task.id === payload.new.id ? payload.new : task
              )
            )
          } else if (payload.eventType === 'DELETE') {
            setTasks(prev =>
              prev.filter(task => task.id !== payload.old.id)
            )
          }
        }
      )
      .subscribe()

    return () => {
      supabase.removeChannel(channel)
    }
  }, [])

  return tasks
}
```

#### 5.2 Robot State Realtime Hook

**`hooks/useRobotStateRealtime.ts`**
```typescript
'use client'

import { useEffect, useState } from 'react'
import { createClient } from '@/lib/supabase/client'

export function useRobotStateRealtime() {
  const [robotState, setRobotState] = useState<any>(null)
  const supabase = createClient()

  useEffect(() => {
    // 초기 데이터 로드
    supabase
      .from('robot_state')
      .select('*')
      .eq('id', 'current')
      .single()
      .then(({ data }) => {
        if (data) setRobotState(data)
      })

    // Realtime 구독
    const channel = supabase
      .channel('robot-state-channel')
      .on(
        'postgres_changes',
        {
          event: 'UPDATE',
          schema: 'public',
          table: 'robot_state',
          filter: 'id=eq.current'
        },
        (payload) => {
          console.log('Robot state updated:', payload.new)
          setRobotState(payload.new)
        }
      )
      .subscribe()

    return () => {
      supabase.removeChannel(channel)
    }
  }, [])

  return robotState
}
```

---

### Phase 6: ROS2 통합

#### 6.1 ROS2 Node (Python)

**`ros2_workspace/src/robot_controller/robot_controller/supabase_listener.py`**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from supabase import create_client, Client
import os
from dotenv import load_dotenv

load_dotenv()

class SupabaseRobotListener(Node):
    def __init__(self):
        super().__init__('supabase_robot_listener')

        # Supabase 클라이언트 초기화
        url = os.environ.get("SUPABASE_URL")
        key = os.environ.get("SUPABASE_SERVICE_ROLE_KEY")
        self.supabase: Client = create_client(url, key)

        self.get_logger().info('🚀 Supabase Robot Listener started')

        # Realtime 구독
        self.subscribe_to_tasks()

    def subscribe_to_tasks(self):
        """tasks 테이블 변경사항 구독"""
        self.supabase.table('tasks').on('INSERT', self.handle_new_task).subscribe()
        self.supabase.table('tasks').on('UPDATE', self.handle_task_update).subscribe()

    def handle_new_task(self, payload):
        """새 작업이 추가되면 실행"""
        task = payload['record']
        self.get_logger().info(f'📋 New task received: {task["id"]}')

        # 작업 상태를 'running'으로 변경
        self.supabase.table('tasks').update({
            'status': 'running',
            'started_at': 'now()'
        }).eq('id', task['id']).execute()

        # 로봇 제어 시작
        self.execute_task(task)

    def execute_task(self, task):
        """로봇 작업 실행"""
        material = task['material_id']
        mode = task['mode_id']

        self.get_logger().info(f'🤖 Executing: {mode} on {material}')

        # TODO: 실제 로봇 제어 코드
        # - 조인트 각도 계산
        # - 모션 플래닝
        # - 작업 실행

        # 진행률 업데이트 예시
        for progress in range(0, 101, 10):
            self.supabase.table('tasks').update({
                'progress': progress
            }).eq('id', task['id']).execute()

            # 로봇 상태 업데이트
            self.update_robot_state(task['id'], progress)

            rclpy.spin_once(self, timeout_sec=1.0)

        # 작업 완료
        self.complete_task(task['id'])

    def update_robot_state(self, task_id, progress):
        """로봇 상태 업데이트"""
        self.supabase.table('robot_state').update({
            'status': 'running',
            'current_task_id': task_id,
            'updated_at': 'now()'
        }).eq('id', 'current').execute()

    def complete_task(self, task_id):
        """작업 완료 처리"""
        # tasks 데이터 가져오기
        task = self.supabase.table('tasks').select('*').eq('id', task_id).single().execute()

        # jobs 테이블로 이동
        self.supabase.table('jobs').insert(task.data).execute()

        # tasks에서 삭제
        self.supabase.table('tasks').delete().eq('id', task_id).execute()

        # 로봇 상태를 idle로 변경
        self.supabase.table('robot_state').update({
            'status': 'idle',
            'current_task_id': None
        }).eq('id', 'current').execute()

        self.get_logger().info(f'✅ Task {task_id} completed')

def main(args=None):
    rclpy.init(args=args)
    node = SupabaseRobotListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS2 패키지 의존성 추가**

`package.xml`:
```xml
<depend>supabase-py</depend>
<depend>python-dotenv</depend>
```

설치:
```bash
pip install supabase python-dotenv
```

---

### Phase 7: Vercel 배포

#### 7.1 프로젝트 설정

**`vercel.json`**
```json
{
  "buildCommand": "npm run build",
  "devCommand": "npm run dev",
  "installCommand": "npm install",
  "framework": "nextjs",
  "outputDirectory": ".next"
}
```

#### 7.2 배포 명령어

```bash
# Vercel CLI 설치
npm install -g vercel

# 로그인
vercel login

# 배포
vercel --prod
```

#### 7.3 환경 변수 설정 (Vercel Dashboard)

1. Vercel 프로젝트 → Settings → Environment Variables
2. 다음 변수 추가:
   - `NEXT_PUBLIC_SUPABASE_URL`
   - `NEXT_PUBLIC_SUPABASE_ANON_KEY`
   - `SUPABASE_SERVICE_ROLE_KEY`

---

## 📊 마이그레이션 체크리스트

### Frontend
- [ ] Next.js 프로젝트 생성
- [ ] 기존 React 컴포넌트 이동
- [ ] React Router → Next.js App Router 변경
- [ ] Supabase 클라이언트 설정
- [ ] Realtime hooks 구현
- [ ] API 호출 경로 수정 (`/api/*` → Next.js API Routes)

### Backend
- [ ] FastAPI 코드 삭제 (더 이상 불필요)
- [ ] Next.js API Routes로 대체
- [ ] Supabase 서버 클라이언트 설정

### Database
- [ ] Supabase 프로젝트 생성
- [ ] PostgreSQL 스키마 생성
- [ ] 초기 데이터 삽입 (materials, modes)
- [ ] Realtime 활성화
- [ ] RLS 정책 설정

### Deployment
- [ ] Vercel 프로젝트 연동
- [ ] 환경 변수 설정
- [ ] 프로덕션 배포
- [ ] 도메인 연결 (선택사항)

### ROS2 Integration
- [ ] ROS2 Node에 Supabase 클라이언트 추가
- [ ] Tasks 테이블 구독
- [ ] 로봇 제어 로직 통합
- [ ] 상태 업데이트 구현

---

## 🔗 참고 자료

- **Supabase 공식 문서**: https://supabase.com/docs
- **Supabase Realtime**: https://supabase.com/docs/guides/realtime
- **Next.js 공식 문서**: https://nextjs.org/docs
- **Vercel 배포 가이드**: https://vercel.com/docs
- **supabase-py (ROS2용)**: https://github.com/supabase-community/supabase-py

---

## 💡 추가 개선사항

### 1. 사용자 인증 추가
```typescript
// Supabase Auth 사용
const { data, error } = await supabase.auth.signInWithPassword({
  email: 'user@example.com',
  password: 'password'
})
```

### 2. 파일 업로드 (로봇 작업 결과 이미지)
```typescript
// Supabase Storage 사용
const { data, error } = await supabase.storage
  .from('job-results')
  .upload(`${job_id}/result.jpg`, file)
```

### 3. Edge Functions (서버리스 함수)
```typescript
// Supabase Edge Functions로 복잡한 로직 처리
// supabase/functions/process-job/index.ts
import { serve } from 'https://deno.land/std@0.168.0/http/server.ts'

serve(async (req) => {
  const { job_id } = await req.json()

  // 복잡한 처리 로직

  return new Response(JSON.stringify({ success: true }), {
    headers: { 'Content-Type': 'application/json' },
  })
})
```

---

## ❓ FAQ

### Q1. 기존 Firebase 데이터를 Supabase로 마이그레이션하려면?

Firebase Admin SDK로 데이터를 추출하고 Supabase에 삽입:

```typescript
// migration-script.ts
import admin from 'firebase-admin'
import { createClient } from '@supabase/supabase-js'

// Firebase 데이터 가져오기
const snapshot = await admin.firestore().collection('materials').get()
const materials = snapshot.docs.map(doc => ({ id: doc.id, ...doc.data() }))

// Supabase에 삽입
const supabase = createClient(SUPABASE_URL, SUPABASE_KEY)
const { error } = await supabase.from('materials').insert(materials)
```

### Q2. WebSocket이 필요한가?

아니요! Supabase Realtime이 WebSocket을 자동으로 처리합니다.

### Q3. Python 백엔드가 완전히 필요 없나?

네, Next.js API Routes로 충분합니다. 단, ROS2 통합 부분만 Python을 사용합니다.

### Q4. 비용은?

- **Supabase 무료 티어**: 500MB 데이터베이스, 1GB 파일 저장소, 50,000 월간 활성 사용자
- **Vercel 무료 티어**: 100GB 대역폭, 무제한 배포

개발 단계에서는 완전 무료입니다!

---

## 🎉 결론

이 마이그레이션을 통해:
1. ✅ Python 환경 문제 완전 해결
2. ✅ 개발/배포 프로세스 단순화
3. ✅ 실시간 데이터 동기화 자동화
4. ✅ 유지보수성 향상
5. ✅ 확장성 확보

**예상 소요 시간**: 2-3일 (풀타임 작업 기준)

추가 질문이 있으면 이슈를 생성하거나 문서를 업데이트해주세요!
