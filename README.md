# 두산 로봇팔 재료 손질 시스템

로봇팔을 활용한 재료 손질 자동화 시스템 웹 애플리케이션

## 프로젝트 구조

```
rokey5_hub1_web/
├── backend/              # FastAPI 백엔드
│   ├── app/
│   │   ├── api/         # API 엔드포인트
│   │   ├── core/        # 설정 및 코어
│   │   ├── models/      # 데이터 모델
│   │   └── services/    # 비즈니스 로직
│   ├── tests/
│   ├── requirements.txt
│   └── README.md
│
├── frontend/            # React + TypeScript 프론트엔드
│   ├── src/
│   │   ├── components/  # 재사용 컴포넌트
│   │   ├── pages/       # 페이지 (MCA-01 ~ MCA-06)
│   │   ├── hooks/       # 커스텀 훅
│   │   ├── services/    # API 클라이언트
│   │   ├── store/       # 상태 관리 (Zustand)
│   │   ├── styles/      # 스타일시트
│   │   └── types/       # 타입 정의
│   ├── public/
│   ├── package.json
│   └── README.md
│
├── shared/              # 공통 타입 정의
│   └── types/
│       ├── material.ts
│       ├── mode.ts
│       ├── job.ts
│       ├── robot.ts
│       ├── path.ts
│       ├── dashboard.ts
│       └── api.ts
│
├── 01_페이지_설명_기획서.md
├── 02_페이지_개발_기획서.md
└── README.md
```

## 시작하기

### 백엔드 실행

```bash
cd backend

# 가상환경 생성 및 활성화
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# 의존성 설치
pip install -r requirements.txt

# 환경변수 설정
cp .env.example .env

# 서버 실행
python -m app.main
# 또는
uvicorn app.main:app --reload
```

API 문서: http://localhost:8000/docs

### 프론트엔드 실행

```bash
cd frontend

# 의존성 설치
npm install

# 개발 서버 실행
npm run dev
```

애플리케이션: http://localhost:5173

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

## 기술 스택

### 백엔드
- **FastAPI** - Python 웹 프레임워크
- **Uvicorn** - ASGI 서버
- **WebSocket** - 실시간 통신
- **Pydantic** - 데이터 검증

### 프론트엔드
- **React 18** - UI 라이브러리
- **TypeScript** - 타입 안정성
- **Vite** - 빌드 도구
- **React Router** - 라우팅
- **Zustand** - 상태 관리
- **Axios** - HTTP 클라이언트
- **Framer Motion** - 애니메이션

## API 엔드포인트

### 재료 (Materials)
- `GET /api/materials` - 재료 목록
- `GET /api/materials/{id}` - 재료 상세

### 모드 (Modes)
- `GET /api/modes` - 모드 목록
- `GET /api/modes/{id}` - 모드 상세

### 작업 (Jobs)
- `POST /api/jobs` - 작업 생성
- `GET /api/jobs/{id}` - 작업 조회
- `POST /api/jobs/{id}/start` - 작업 시작
- `POST /api/jobs/{id}/pause` - 일시정지
- `POST /api/jobs/{id}/resume` - 재개
- `POST /api/jobs/{id}/stop` - 정지
- `WS /api/jobs/ws/{id}` - 실시간 모니터링

### 로봇 (Robot)
- `GET /api/robot/status` - 상태 조회
- `POST /api/robot/emergency-stop` - 비상 정지
- `POST /api/robot/release-emergency` - 비상 정지 해제
- `WS /api/robot/ws/status` - 실시간 상태

### 대시보드 (Dashboard)
- `GET /api/dashboard` - 대시보드 데이터

### 경로 (Paths)
- `GET /api/paths` - 경로 목록
- `POST /api/paths` - 경로 생성
- `PUT /api/paths/{id}` - 경로 수정
- `DELETE /api/paths/{id}` - 경로 삭제

## 개발 일정

- **Week 1**: 기본 UI 구조 및 작업 설정 플로우
- **Week 2**: 작업 모니터링 및 상태 관리
- **Week 3**: 로봇 상태 페이지 및 경로 설정
- **Week 4**: 통합 테스트 및 버그 수정

## 참고 문서

- [페이지 설명 기획서](./01_페이지_설명_기획서.md)
- [페이지 개발 기획서](./02_페이지_개발_기획서.md)
- [백엔드 README](./backend/README.md)
- [프론트엔드 README](./frontend/README.md)
- [공통 타입 README](./shared/README.md)

## 라이선스

MIT