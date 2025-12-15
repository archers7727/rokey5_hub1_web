# 공통 타입 정의 (Shared Types)

백엔드와 프론트엔드에서 공유하는 TypeScript 타입 정의

## 구조

```
shared/
└── types/
    ├── material.ts     # 재료 관련 타입
    ├── mode.ts         # 손질 모드 관련 타입
    ├── job.ts          # 작업 관련 타입
    ├── robot.ts        # 로봇 상태 관련 타입
    ├── path.ts         # 경로 설정 관련 타입
    ├── dashboard.ts    # 대시보드 관련 타입
    ├── api.ts          # API 응답 관련 타입
    └── index.ts        # 전체 Export
```

## 사용 방법

### 프론트엔드 (TypeScript)

```typescript
import {
  Material,
  Mode,
  Job,
  RobotState,
  Path,
} from '../../../shared/types'

// 또는 타입별로 임포트
import { MaterialSize } from '../../../shared/types/material'
import { JobStatus } from '../../../shared/types/job'
```

### 백엔드 (Python with Pydantic)

TypeScript 타입을 참고하여 Pydantic 모델 생성:

```python
from pydantic import BaseModel
from typing import Literal

class Material(BaseModel):
    id: str
    name: str
    emoji: str
    description: str
    category: Literal['vegetable', 'fruit', 'meat']
    sizes: dict[str, float]
```

## 타입 정의

### Material (재료)
- `Material`: 재료 기본 정보
- `MaterialCategory`: 재료 카테고리
- `MaterialSize`: 재료 크기 (small, medium, large)

### Mode (손질 모드)
- `Mode`: 모드 기본 정보
- `ModeType`: 모드 타입 (frying, slicing)
- `ModeParameter`: 모드 파라미터 정의
- `FryingModeParams`: 튀김 모드 파라미터
- `SlicingModeParams`: 썰기 모드 파라미터

### Job (작업)
- `Job`: 작업 전체 정보
- `JobStatus`: 작업 상태
- `JobParameters`: 작업 파라미터
- `CreateJobRequest`: 작업 생성 요청
- `JobMonitoringData`: 작업 모니터링 데이터

### Robot (로봇)
- `RobotState`: 로봇 전체 상태
- `RobotStatus`: 로봇 상태
- `TCPPosition`: TCP 위치/자세
- `JointInfo`: 관절 정보

### Path (경로)
- `Path`: 경로 전체 정보
- `PathPoint`: 경로 포인트
- `PointType`: 포인트 타입
- `PathValidationResult`: 경로 검증 결과

### Dashboard (대시보드)
- `DashboardData`: 대시보드 데이터
- `TodayStats`: 오늘의 통계
- `RecentJob`: 최근 작업

### API (응답)
- `ApiResponse<T>`: 표준 API 응답
- `PaginatedResponse<T>`: 페이지네이션 응답

## 타입 동기화

1. TypeScript 타입이 단일 진실 공급원(Single Source of Truth)
2. 백엔드는 TypeScript 타입을 참고하여 Pydantic 모델 생성
3. 타입 변경 시 양쪽 모두 업데이트 필요

## 향후 개선

- [ ] 타입 자동 생성 스크립트 (TypeScript → Python)
- [ ] 타입 검증 도구
- [ ] 버전 관리
