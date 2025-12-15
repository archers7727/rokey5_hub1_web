# 배포 가이드

## 백엔드 배포 (Render.com)

1. **Render.com 계정 생성**
   - https://render.com 접속
   - GitHub 계정으로 로그인

2. **새 Web Service 생성**
   - "New +" 버튼 클릭 → "Web Service" 선택
   - GitHub 저장소 연결
   - 저장소 선택: `rokey5_hub1_web`

3. **설정 입력**
   - **Name**: `rokey5-hub1-backend`
   - **Region**: `Oregon (US West)`
   - **Branch**: `claude/plan-dev-workflow-TdA8A` (또는 main)
   - **Root Directory**: `backend`
   - **Runtime**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

4. **환경 변수 설정**
   - `PYTHON_VERSION`: `3.11.0`
   - `MOCK_ROBOT`: `true`

5. **배포 완료**
   - "Create Web Service" 클릭
   - 배포 완료 후 URL 복사 (예: `https://rokey5-hub1-backend.onrender.com`)

## 프론트엔드 배포 (Vercel)

1. **백엔드 URL 업데이트**
   - Vercel 대시보드에서 프로젝트 선택
   - Settings → Environment Variables
   - 새 환경 변수 추가:
     - **Name**: `VITE_API_BASE_URL`
     - **Value**: `https://your-backend-url.onrender.com/api`
   - 모든 환경(Production, Preview, Development)에 적용

2. **재배포**
   - Deployments 탭으로 이동
   - 최신 배포의 "..." 메뉴 → "Redeploy"

3. **vercel.json 업데이트** (선택사항)
   - 백엔드 URL이 변경되면 `vercel.json`의 `rewrites` 섹션 업데이트:
   ```json
   {
     "source": "/api/:path*",
     "destination": "https://your-backend-url.onrender.com/api/:path*"
   }
   ```

## 로컬 개발 환경

### 백엔드 실행
```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### 프론트엔드 실행
```bash
cd frontend
npm install
npm run dev
```

프론트엔드는 `http://localhost:5173`에서 실행되며,
`/api` 요청은 자동으로 `http://localhost:8000`으로 프록시됩니다.

## 트러블슈팅

### Vercel 404 오류
- `vercel.json`의 rewrites 설정 확인
- SPA 라우팅을 위해 모든 경로가 `/index.html`로 리다이렉트되는지 확인

### API 연결 오류
- 프론트엔드 환경 변수 `VITE_API_BASE_URL` 확인
- 백엔드 CORS 설정 확인 (`app/main.py`)
- 백엔드가 정상적으로 실행 중인지 확인

### 백엔드 배포 실패
- Python 버전 확인 (3.11.0)
- requirements.txt의 모든 패키지가 설치되는지 확인
- Render 로그에서 오류 메시지 확인
