# 두산 로봇팔 재료 손질 시스템

로봇팔을 활용한 재료 손질 자동화 시스템 웹 애플리케이션

## 🚀 최신 아키텍처 (Next.js + Supabase)

프로젝트가 **Next.js + Supabase**로 마이그레이션되었습니다!

- ✅ Python 의존성 문제 해결
- ✅ 프론트엔드 + 백엔드 통합 배포 (Vercel)
- ✅ 실시간 데이터 동기화 (Supabase Realtime)
- ✅ 무료 티어로 개발 가능

> **마이그레이션 가이드**: [MIGRATION_TO_VERCEL_SUPABASE.md](./MIGRATION_TO_VERCEL_SUPABASE.md)

## 프로젝트 구조

```
rokey5_hub1_web/
├── frontend-next/       # ⭐ Next.js 프론트엔드 (새 아키텍처)
│   ├── app/
│   │   ├── api/        # API Routes (serverless functions)
│   │   ├── layout.tsx  # Root layout
│   │   └── page.tsx    # Home page
│   ├── components/     # 재사용 컴포넌트
│   ├── hooks/          # Realtime hooks
│   ├── lib/
│   │   └── supabase/   # Supabase client
│   ├── types/          # TypeScript 타입
│   └── README.md
│
├── frontend/           # [Legacy] React + Vite (곧 제거 예정)
├── backend/            # [Legacy] FastAPI (곧 제거 예정)
│
├── shared/             # 공통 타입 정의
│   └── types/
│
├── 01_페이지_설명_기획서.md
├── 02_페이지_개발_기획서.md
├── MIGRATION_TO_VERCEL_SUPABASE.md
└── README.md
```

## 🚀 빠른 시작

### 1. Supabase 프로젝트 설정

1. [Supabase](https://supabase.com) 가입 및 프로젝트 생성
2. SQL Editor에서 [마이그레이션 가이드의 스키마](./MIGRATION_TO_VERCEL_SUPABASE.md#31-supabase-sql-editor에서-실행) 실행
3. Project Settings > API에서 URL과 API 키 복사

### 2. 환경 변수 설정

```bash
cd frontend-next
cp .env.example .env.local
```

`.env.local` 파일을 열어서 Supabase 정보 입력:

```env
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
SUPABASE_SERVICE_ROLE_KEY=your-service-role-key
```

### 3. 애플리케이션 실행

```bash
cd frontend-next

# 의존성 설치
npm install

# 개발 서버 실행
npm run dev
```

애플리케이션: http://localhost:3000

### 4. Vercel 배포 (선택사항)

```bash
# Vercel CLI 설치
npm install -g vercel

# 배포
vercel --prod
```

> Vercel 대시보드에서 환경 변수를 설정해야 합니다!

## 주요 기능

### 1차 개발 범위 (MCA-01 ~ MCA-06)

- **MCA-01**: 메인 대시보드
- **MCA-02**: 작업 설정 플로우 (4단계)
  - MCA-02-01: 재료 선택
  - MCA-02-02: 손질 모드 선택
  - MCA-02-03: 작업 파라미터 설정
  - MCA-02-04: 작업 확인 및 미리보기
- **MCA-03**: 작업 실행 모니터링
- **MCA-04**: 작업 완료
- **MCA-05**: 로봇 상태 페이지
- **MCA-06**: 작업 경로 설정

> **Note**: 3D 시각화 기능은 1차 개발에서 제외됨

## 🛠️ 기술 스택

### Frontend & Backend (통합)
- **Next.js 15** - React 프레임워크 (App Router)
- **TypeScript** - 타입 안정성
- **Tailwind CSS** - 유틸리티 CSS 프레임워크
- **Supabase** - PostgreSQL 데이터베이스 + Realtime
- **Supabase Realtime** - WebSocket 기반 실시간 업데이트
- **Vercel** - 서버리스 배포 플랫폼
- **Zustand** - 상태 관리
- **Framer Motion** - 애니메이션

### Legacy (곧 제거 예정)
- ~~FastAPI + Python 백엔드~~
- ~~React + Vite 프론트엔드~~
- ~~Firebase Firestore~~

## 📡 API Routes (Next.js Serverless Functions)

### 재료 (Materials)
- `GET /api/materials` - 재료 목록 조회

### 모드 (Modes)
- `GET /api/modes?material={id}` - 모드 목록 (재료별 필터링)

### 작업 (Jobs)
- `GET /api/jobs` - 작업 히스토리 조회 (최근 50개)
- `POST /api/jobs` - 새 작업 생성 (tasks 테이블에 추가)

### 로봇 (Robot)
- `GET /api/robot/state` - 현재 로봇 상태 조회

## 🔄 Realtime Subscriptions (Supabase)

### Tasks (작업 큐)
- `useTasksRealtime()` - 작업 큐 실시간 업데이트
- INSERT, UPDATE, DELETE 이벤트 자동 반영

### Robot State (로봇 상태)
- `useRobotStateRealtime()` - 로봇 상태 실시간 업데이트
- 조인트 각도, TCP 위치, 에러 상태 등

## 📚 데이터베이스 스키마 (Supabase)

### Tables
1. **materials** - 재료 정보 (양파, 감자 등)
2. **modes** - 손질 모드 (튀김, 썰기 등)
3. **tasks** - 작업 큐 (pending, running, completed)
4. **robot_state** - 로봇 실시간 상태
5. **jobs** - 완료된 작업 히스토리

### Realtime 활성화
- `tasks` 테이블 - 작업 큐 변경사항 실시간 구독
- `robot_state` 테이블 - 로봇 상태 실시간 구독

전체 스키마는 [마이그레이션 가이드](./MIGRATION_TO_VERCEL_SUPABASE.md#phase-3-데이터베이스-스키마)를 참고하세요.

## 📋 다음 단계

1. ✅ Next.js 프로젝트 생성 및 Supabase 설정
2. 🔄 기존 React 컴포넌트를 Next.js로 마이그레이션
3. 🔄 페이지 라우팅 구현 (Dashboard, MaterialSelection 등)
4. 🔄 ROS2 통합 (Supabase Realtime 구독)

## 📖 참고 문서

- [Vercel + Supabase 마이그레이션 가이드](./MIGRATION_TO_VERCEL_SUPABASE.md) ⭐
- [페이지 설명 기획서](./01_페이지_설명_기획서.md)
- [페이지 개발 기획서](./02_페이지_개발_기획서.md)
- [Next.js 프론트엔드 README](./frontend-next/README.md)
- [Legacy 백엔드 README](./backend/README.md)
- [Legacy 프론트엔드 README](./frontend/README.md)

## 라이선스

MIT