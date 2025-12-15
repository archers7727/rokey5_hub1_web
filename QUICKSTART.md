# 빠른 시작 가이드

## 배포 요약 (3단계)

### 1️⃣ 백엔드 배포 (Cloud Run)

```bash
# 1. Google Cloud 로그인 및 프로젝트 설정
gcloud auth login
gcloud config set project YOUR_PROJECT_ID

# 2. API 활성화
gcloud services enable run.googleapis.com cloudbuild.googleapis.com

# 3. 백엔드 배포
cd backend
gcloud run deploy rokey5-backend \
  --source . \
  --platform managed \
  --region us-central1 \
  --allow-unauthenticated \
  --set-env-vars MOCK_ROBOT=true

# 배포 완료 후 URL 복사! (예: https://rokey5-backend-xxxxx.run.app)
```

### 2️⃣ Firebase 설정

```bash
# 1. Firebase 로그인
firebase login

# 2. .firebaserc 파일에서 프로젝트 ID 업데이트
# "your-firebase-project-id" → 실제 프로젝트 ID로 변경

# 3. Firebase 초기화 (선택사항 - 이미 설정되어 있음)
firebase init hosting
```

### 3️⃣ 프론트엔드 배포 (Firebase Hosting)

```bash
# 1. 프론트엔드 빌드
cd frontend
npm install
npm run build

# 2. Firebase에 배포
cd ..
firebase deploy --only hosting

# 배포 완료! URL에서 확인하세요
```

---

## 로컬 개발

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

로컬 접속: http://localhost:5173

---

## 체크리스트

### 배포 전 확인사항
- [ ] Google Cloud 프로젝트 생성
- [ ] Firebase 프로젝트 연결
- [ ] gcloud CLI 설치 및 로그인
- [ ] firebase CLI 설치 및 로그인
- [ ] `.firebaserc`에서 프로젝트 ID 업데이트

### 배포 후 확인사항
- [ ] Cloud Run 서비스 URL 확인
- [ ] Firebase Hosting URL 접속 테스트
- [ ] API 엔드포인트 작동 확인 (`/api/materials`)
- [ ] 전체 작업 플로우 테스트 (재료 선택 → 완료)

---

## 문제 해결

### Cloud Run 배포 실패
```bash
# 로그 확인
gcloud run services logs read rokey5-backend --region us-central1

# Docker 로컬 테스트
cd backend
docker build -t rokey5-backend .
docker run -p 8080:8080 -e MOCK_ROBOT=true rokey5-backend
```

### Firebase 배포 실패
```bash
# 빌드 파일 확인
ls -la frontend/dist

# 로그 확인
firebase deploy --only hosting --debug
```

### 404 오류
- `firebase.json`의 rewrites 설정 확인
- Cloud Run 서비스 이름이 `rokey5-backend`인지 확인
- SPA 라우팅이 `index.html`로 리다이렉트되는지 확인

---

## 다음 단계

배포가 완료되면:
1. ✅ 포트폴리오에 URL 추가
2. ✅ 실제 로봇 연동 준비 (`MOCK_ROBOT=false`)
3. ✅ MCA-05, MCA-06 페이지 구현
4. ✅ 3D 시각화 추가
5. ✅ 모니터링 및 로깅 설정 (Google Cloud Logging)

전체 가이드는 `DEPLOYMENT.md`를 참고하세요!
