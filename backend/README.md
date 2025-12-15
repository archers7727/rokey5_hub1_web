# 두산 로봇팔 재료 손질 시스템 - Backend API

FastAPI 기반 로봇팔 제어 및 작업 관리 REST API

## 시작하기

### 1. 가상환경 생성 및 활성화

```bash
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

### 2. 의존성 설치

```bash
pip install -r requirements.txt
```

### 3. 환경변수 설정

```bash
cp .env.example .env
# .env 파일을 편집하여 필요한 설정 변경
```

### 4. 서버 실행

```bash
# 개발 모드 (자동 리로드)
python -m app.main

# 또는
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### 5. API 문서 확인

- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## API 엔드포인트

### 재료 (Materials)
- `GET /api/materials` - 모든 재료 조회
- `GET /api/materials/{material_id}` - 특정 재료 조회

### 모드 (Modes)
- `GET /api/modes` - 모든 모드 조회
- `GET /api/modes?material=onion` - 재료별 모드 조회
- `GET /api/modes/{mode_id}` - 특정 모드 조회

### 작업 (Jobs)
- `POST /api/jobs` - 작업 생성
- `GET /api/jobs` - 작업 목록 조회
- `GET /api/jobs/{job_id}` - 작업 조회
- `POST /api/jobs/{job_id}/start` - 작업 시작
- `POST /api/jobs/{job_id}/pause` - 작업 일시정지
- `POST /api/jobs/{job_id}/resume` - 작업 재개
- `POST /api/jobs/{job_id}/stop` - 작업 정지
- `POST /api/jobs/{job_id}/speed` - 작업 속도 변경
- `WS /api/jobs/ws/{job_id}` - 작업 실시간 모니터링

### 로봇 (Robot)
- `GET /api/robot/status` - 로봇 상태 조회
- `POST /api/robot/emergency-stop` - 비상 정지
- `POST /api/robot/release-emergency` - 비상 정지 해제
- `POST /api/robot/resume` - 재개
- `POST /api/robot/stop` - 정지
- `WS /api/robot/ws/status` - 로봇 상태 실시간 모니터링

### 대시보드 (Dashboard)
- `GET /api/dashboard` - 대시보드 데이터 조회

### 경로 (Paths)
- `GET /api/paths` - 모든 경로 조회
- `GET /api/paths/{path_id}` - 특정 경로 조회
- `POST /api/paths` - 경로 생성
- `PUT /api/paths/{path_id}` - 경로 수정
- `DELETE /api/paths/{path_id}` - 경로 삭제
- `POST /api/paths/{path_id}/validate` - 경로 검증

## 프로젝트 구조

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI 앱 진입점
│   ├── api/
│   │   ├── __init__.py
│   │   ├── router.py        # API 라우터 통합
│   │   └── endpoints/       # 각 엔드포인트
│   │       ├── materials.py
│   │       ├── modes.py
│   │       ├── jobs.py
│   │       ├── robot.py
│   │       ├── dashboard.py
│   │       └── paths.py
│   ├── core/
│   │   ├── __init__.py
│   │   └── config.py        # 설정 관리
│   ├── models/              # 데이터 모델
│   └── services/            # 비즈니스 로직
├── tests/                   # 테스트
├── requirements.txt
├── .env.example
└── README.md
```

## 개발

### 테스트 실행

```bash
pytest
```

### 코드 포맷팅

```bash
black app/
```

### 린팅

```bash
flake8 app/
```
