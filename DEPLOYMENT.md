# 배포 가이드 (Firebase Hosting + Cloud Run)

## 사전 준비

### 1. Google Cloud 및 Firebase 설정

1. **Google Cloud Console**에서 프로젝트 생성
   - https://console.cloud.google.com
   - 새 프로젝트 생성 (예: `rokey5-hub1-web`)
   - 프로젝트 ID 복사 (나중에 필요)

2. **Firebase 프로젝트 연결**
   - https://console.firebase.google.com
   - "프로젝트 추가" → 위에서 만든 Google Cloud 프로젝트 선택
   - Google Analytics는 선택사항

3. **필수 도구 설치** (로컬)
```bash
# Firebase CLI
npm install -g firebase-tools

# Google Cloud SDK
# macOS
brew install google-cloud-sdk

# Linux
curl https://sdk.cloud.google.com | bash

# Windows
# https://cloud.google.com/sdk/docs/install 참고
```

4. **로그인**
```bash
# Firebase 로그인
firebase login

# Google Cloud 로그인
gcloud auth login
gcloud config set project YOUR_PROJECT_ID
```

---

## 백엔드 배포 (Google Cloud Run)

### 1단계: 프로젝트 설정

```bash
# 프로젝트 ID 설정
gcloud config set project YOUR_PROJECT_ID

# Cloud Run API 활성화
gcloud services enable run.googleapis.com
gcloud services enable cloudbuild.googleapis.com
```

### 2단계: 백엔드 배포

```bash
cd backend

# Cloud Run에 배포 (자동으로 Docker 이미지 빌드 및 배포)
gcloud run deploy rokey5-backend \
  --source . \
  --platform managed \
  --region us-central1 \
  --allow-unauthenticated \
  --set-env-vars MOCK_ROBOT=true
```

배포가 완료되면 **서비스 URL**이 표시됩니다:
```
Service URL: https://rokey5-backend-xxxxx-uc.a.run.app
```

이 URL을 복사해두세요! (Firebase 설정에 필요)

### 3단계: Cloud Run 설정 확인

```bash
# 배포된 서비스 확인
gcloud run services list

# 서비스 세부정보 확인
gcloud run services describe rokey5-backend --region us-central1
```

---

## 프론트엔드 배포 (Firebase Hosting)

### 1단계: Firebase 프로젝트 초기화

```bash
# 프로젝트 루트 디렉토리에서
firebase init

# 선택사항:
# - Hosting (Space로 선택)
# - 기존 프로젝트 사용
# - Public directory: frontend/dist
# - Single-page app: Yes
# - GitHub 자동 배포: No (선택사항)
```

### 2단계: .firebaserc 업데이트

`.firebaserc` 파일에서 프로젝트 ID 업데이트:
```json
{
  "projects": {
    "default": "YOUR_PROJECT_ID"
  }
}
```

### 3단계: firebase.json 업데이트

`firebase.json`에서 Cloud Run 서비스 ID 확인:
```json
{
  "hosting": {
    "rewrites": [
      {
        "source": "/api/**",
        "run": {
          "serviceId": "rokey5-backend",  ← 여기가 Cloud Run 서비스 이름
          "region": "us-central1"
        }
      }
    ]
  }
}
```

### 4단계: 프론트엔드 빌드 및 배포

```bash
cd frontend

# 의존성 설치
npm install

# 프로덕션 빌드
npm run build

# Firebase에 배포
cd ..
firebase deploy --only hosting
```

배포 완료 후 URL이 표시됩니다:
```
Hosting URL: https://YOUR_PROJECT_ID.web.app
```

---

## 환경 변수 설정

### 프론트엔드 환경 변수

프로덕션에서는 Firebase Hosting이 자동으로 `/api` 요청을 Cloud Run으로 라우팅하므로,
**환경 변수가 필요하지 않습니다**.

하지만 로컬 개발을 위해 `.env.local` 파일 생성:
```bash
# frontend/.env.local
VITE_API_BASE_URL=http://localhost:8000/api
```

### 백엔드 환경 변수

Cloud Run에 환경 변수 추가/수정:
```bash
gcloud run services update rokey5-backend \
  --region us-central1 \
  --set-env-vars MOCK_ROBOT=true,OTHER_VAR=value
```

---

## 무료 한도 (매월)

### Google Cloud Run
- ✅ **요청**: 200만 건
- ✅ **컴퓨팅**: 180,000 vCPU-초
- ✅ **메모리**: 360,000 GiB-초
- ✅ **네트워크**: 1 GB 송신

### Firebase Hosting
- ✅ **저장소**: 10 GB
- ✅ **전송량**: 360 MB/일
- ✅ **SSL**: 무료

**포트폴리오용으로는 충분히 넉넉합니다!**

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
