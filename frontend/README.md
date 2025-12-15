# 두산 로봇팔 재료 손질 시스템 - Frontend

React + TypeScript + Vite 기반 프론트엔드 애플리케이션

## 시작하기

### 1. 의존성 설치

```bash
npm install
# 또는
yarn install
# 또는
pnpm install
```

### 2. 개발 서버 실행

```bash
npm run dev
```

브라우저에서 http://localhost:5173 접속

### 3. 빌드

```bash
npm run build
```

### 4. 프리뷰

```bash
npm run preview
```

## 프로젝트 구조

```
frontend/
├── public/              # 정적 파일
├── src/
│   ├── components/      # 재사용 가능한 컴포넌트
│   │   └── Layout.tsx   # 레이아웃 컴포넌트
│   ├── pages/           # 페이지 컴포넌트
│   │   ├── Dashboard.tsx                 # MCA-01
│   │   ├── MaterialSelection.tsx         # MCA-02-01
│   │   ├── ModeSelection.tsx             # MCA-02-02
│   │   ├── ParameterConfiguration.tsx    # MCA-02-03
│   │   ├── JobConfirmation.tsx           # MCA-02-04
│   │   ├── JobMonitoring.tsx             # MCA-03
│   │   ├── JobCompletion.tsx             # MCA-04
│   │   ├── RobotStatus.tsx               # MCA-05
│   │   └── PathConfiguration.tsx         # MCA-06
│   ├── hooks/           # 커스텀 훅
│   │   └── useWebSocket.ts
│   ├── services/        # API 클라이언트
│   │   └── api.ts
│   ├── store/           # 상태 관리 (Zustand)
│   │   ├── jobConfigStore.ts
│   │   └── robotStore.ts
│   ├── types/           # TypeScript 타입 정의
│   ├── styles/          # 스타일시트
│   │   └── index.css    # 전역 스타일 + 디자인 시스템
│   ├── App.tsx          # 앱 루트 + 라우팅
│   └── main.tsx         # 진입점
├── index.html
├── vite.config.ts
├── tsconfig.json
└── package.json
```

## 페이지 라우팅

| 경로 | 페이지 | 설명 |
|------|--------|------|
| `/` | Dashboard | MCA-01: 메인 대시보드 |
| `/job/new/material` | MaterialSelection | MCA-02-01: 재료 선택 |
| `/job/new/mode` | ModeSelection | MCA-02-02: 모드 선택 |
| `/job/new/parameters` | ParameterConfiguration | MCA-02-03: 파라미터 설정 |
| `/job/new/confirm` | JobConfirmation | MCA-02-04: 작업 확인 |
| `/job/monitor/:jobId` | JobMonitoring | MCA-03: 작업 모니터링 |
| `/job/complete/:jobId` | JobCompletion | MCA-04: 작업 완료 |
| `/robot/status` | RobotStatus | MCA-05: 로봇 상태 |
| `/path/editor` | PathConfiguration | MCA-06: 경로 설정 |

## 기술 스택

- **React 18** - UI 라이브러리
- **TypeScript** - 타입 안정성
- **Vite** - 빌드 도구
- **React Router** - 라우팅
- **Zustand** - 상태 관리
- **Axios** - HTTP 클라이언트
- **Framer Motion** - 애니메이션 (옵션)

## 디자인 시스템

CSS Variables를 사용한 일관된 디자인 시스템:

- 컬러: Primary, Status, Neutral
- 타이포그래피: Font sizes, weights
- 스페이싱: 4px 기준
- Border Radius, Shadows
- 애니메이션 duration, easing

자세한 내용은 `src/styles/index.css` 참조

## API 연동

백엔드 API는 Vite 프록시를 통해 연결:

- API: `http://localhost:8000/api` → `/api`
- WebSocket: `ws://localhost:8000/ws` → `/ws`

`src/services/api.ts`에서 모든 API 함수 정의

## 상태 관리

### 작업 설정 플로우 (jobConfigStore)
```typescript
import { useJobConfigStore } from '@store/jobConfigStore'

const { material, setMaterial } = useJobConfigStore()
```

### 로봇 상태 (robotStore)
```typescript
import { useRobotStore } from '@store/robotStore'

const { status, setStatus } = useRobotStore()
```

## WebSocket 사용

```typescript
import { useWebSocket } from '@hooks/useWebSocket'

const { isConnected, lastMessage } = useWebSocket('/api/robot/ws/status', {
  onMessage: (data) => console.log(data),
})
```

## 개발 노트

- 3D 시각화는 1차 개발에서 제외
- 페이지 기본 구조만 생성됨
- 실제 UI 구현은 순차적으로 진행
