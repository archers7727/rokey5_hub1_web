"""
Firestore 데이터베이스 연결 및 초기화
"""
import os
import firebase_admin
from firebase_admin import credentials, firestore
from .config import settings

# Firebase 초기화 상태
_firebase_initialized = False
db = None


def initialize_firebase():
    """Firebase Admin SDK 초기화"""
    global _firebase_initialized, db

    if _firebase_initialized:
        return db

    try:
        # Service Account 키 파일 경로
        cred_path = settings.FIREBASE_CREDENTIALS_PATH

        # 환경 변수로도 설정 가능
        if os.getenv('FIREBASE_CREDENTIALS_PATH'):
            cred_path = os.getenv('FIREBASE_CREDENTIALS_PATH')

        # 파일 존재 확인
        if not os.path.exists(cred_path):
            print(f"⚠️  Firebase credentials not found at: {cred_path}")
            print(f"⚠️  Running in MOCK mode without Firestore")
            return None

        # Firebase Admin SDK 초기화
        cred = credentials.Certificate(cred_path)
        firebase_admin.initialize_app(cred, {
            'projectId': settings.FIREBASE_PROJECT_ID,
        })

        # Firestore 클라이언트
        db = firestore.client()
        _firebase_initialized = True

        print(f"✅ Firebase initialized successfully")
        print(f"✅ Project ID: {settings.FIREBASE_PROJECT_ID}")

        return db

    except Exception as e:
        print(f"❌ Firebase initialization failed: {e}")
        print(f"⚠️  Running in MOCK mode without Firestore")
        return None


def get_db():
    """Firestore 클라이언트 가져오기"""
    global db
    if db is None:
        db = initialize_firebase()
    return db


# 컬렉션 참조 헬퍼 함수
def get_collection(collection_name: str):
    """컬렉션 참조 가져오기"""
    database = get_db()
    if database is None:
        return None
    return database.collection(collection_name)


# 자주 사용하는 컬렉션 참조
def get_tasks_ref():
    """tasks 컬렉션 참조"""
    return get_collection('tasks')


def get_robot_state_ref():
    """robot_state 컬렉션 참조"""
    return get_collection('robot_state')


def get_jobs_ref():
    """jobs 컬렉션 참조"""
    return get_collection('jobs')


def get_materials_ref():
    """materials 컬렉션 참조"""
    return get_collection('materials')


def get_modes_ref():
    """modes 컬렉션 참조"""
    return get_collection('modes')


def get_paths_ref():
    """paths 컬렉션 참조"""
    return get_collection('paths')
