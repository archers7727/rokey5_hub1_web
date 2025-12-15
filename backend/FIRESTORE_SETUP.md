# Firestore 설정 가이드

## 1. Service Account 키 파일 배치

### 로컬 개발 환경

1. **키 파일을 다운로드했으면**, 다음 위치에 배치:
   ```
   backend/config/serviceAccountKey.json
   ```

2. **디렉토리 생성** (없다면):
   ```bash
   cd backend
   mkdir -p config
   ```

3. **키 파일 복사**:
   ```bash
   # Windows
   copy C:\Users\yourname\Downloads\rokey-test-481307-*.json config\serviceAccountKey.json

   # Mac/Linux
   cp ~/Downloads/rokey-test-481307-*.json config/serviceAccountKey.json
   ```

4. **권한 설정** (Mac/Linux만):
   ```bash
   chmod 600 config/serviceAccountKey.json
   ```

### ⚠️ 중요: Git에 절대 커밋하지 마세요!

`.gitignore`에 이미 다음 패턴이 추가되어 있습니다:
```
**/serviceAccountKey.json
**/*-firebase-adminsdk-*.json
backend/config/
```

**확인 방법**:
```bash
git status
# config/ 폴더가 Untracked files에 나타나지 않아야 합니다
```

---

## 2. 환경 변수 설정 (선택사항)

`.env` 파일 생성 (또는 `.env.example` 복사):

```bash
cd backend
cp .env.example .env
```

`.env` 파일에서 필요시 경로 수정:
```env
FIREBASE_CREDENTIALS_PATH=config/serviceAccountKey.json
FIREBASE_PROJECT_ID=rokey-test-481307
```

---

## 3. 초기 데이터 삽입

Firestore에 Materials, Modes, Robot State 초기 데이터를 삽입합니다.

```bash
cd backend

# 의존성 설치 (firebase-admin 포함)
pip install -r requirements.txt

# 초기 데이터 스크립트 실행
python scripts/init_firestore.py
```

**예상 출력**:
```
==================================================
🚀 Firestore 초기 데이터 삽입
==================================================

✅ Firebase initialized successfully
✅ Project ID: rokey-test-481307

📦 Initializing materials...
  ✅ Added: 양파 (onion)
  ✅ Added: 감자 (potato)
✅ Materials initialized (2 items)

🔧 Initializing modes...
  ✅ Added: 튀김 (frying)
  ✅ Added: 썰기 (slicing)
✅ Modes initialized (2 items)

🤖 Initializing robot state...
✅ Robot state initialized

==================================================
✅ 모든 데이터가 성공적으로 초기화되었습니다!
==================================================
```

---

## 4. Firestore Console에서 확인

1. **Firebase Console** 접속:
   - https://console.firebase.google.com/project/rokey-test-481307/firestore

2. **Collections 확인**:
   - `materials` → onion, potato 문서 확인
   - `modes` → frying, slicing 문서 확인
   - `robot_state` → current 문서 확인

---

## 5. Cloud Run 배포 시 설정

### 방법 1: Secret Manager 사용 (권장)

```bash
# 1. Secret 생성
gcloud secrets create firebase-credentials \
  --data-file=config/serviceAccountKey.json

# 2. Cloud Run에 Secret 마운트
gcloud run deploy rokey5-backend \
  --source . \
  --region us-central1 \
  --update-secrets=/secrets/firebase-credentials=firebase-credentials:latest

# 3. 환경 변수 설정
gcloud run services update rokey5-backend \
  --region us-central1 \
  --set-env-vars FIREBASE_CREDENTIALS_PATH=/secrets/firebase-credentials
```

### 방법 2: 환경 변수로 JSON 전달 (비권장)

```bash
# Service Account 키를 base64로 인코딩
cat config/serviceAccountKey.json | base64

# 환경 변수로 설정
gcloud run services update rokey5-backend \
  --region us-central1 \
  --set-env-vars FIREBASE_CREDENTIALS_BASE64="<base64-encoded-json>"
```

---

## 트러블슈팅

### 오류: "Firebase credentials not found"

```
⚠️  Firebase credentials not found at: config/serviceAccountKey.json
⚠️  Running in MOCK mode without Firestore
```

**해결 방법**:
1. `backend/config/` 디렉토리가 존재하는지 확인
2. `serviceAccountKey.json` 파일이 올바른 위치에 있는지 확인
3. 파일 이름이 정확한지 확인 (대소문자 구분)

### 오류: "Permission denied"

```bash
chmod 600 config/serviceAccountKey.json
```

### 오류: "Project ID mismatch"

`.env` 파일에서 `FIREBASE_PROJECT_ID`가 실제 프로젝트 ID와 일치하는지 확인:
```env
FIREBASE_PROJECT_ID=rokey-test-481307
```

---

## 디렉토리 구조

```
backend/
├── config/                          # ← Git에 커밋되지 않음!
│   └── serviceAccountKey.json       # ← Service Account 키
├── scripts/
│   └── init_firestore.py            # 초기 데이터 스크립트
├── app/
│   └── core/
│       ├── config.py                # 설정 관리
│       └── firestore.py             # Firestore 연결
├── .env                             # ← Git에 커밋되지 않음!
├── .env.example                     # 환경 변수 템플릿
└── requirements.txt                 # firebase-admin 포함
```

---

## 다음 단계

Firestore 설정이 완료되면:
1. ✅ Backend API 엔드포인트를 Firestore와 연동
2. ✅ 로컬에서 테스트
3. ✅ Cloud Run에 배포

설정이 완료되었으면 다음 명령어로 백엔드를 실행하세요:
```bash
cd backend
uvicorn app.main:app --reload
```
