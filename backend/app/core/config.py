"""
애플리케이션 설정 관리
"""
from pydantic import BaseSettings
from typing import List


class Settings(BaseSettings):
    """애플리케이션 설정"""

    # API 설정
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000
    API_RELOAD: bool = True
    API_VERSION: str = "v1"
    API_PREFIX: str = "/api"

    # CORS 설정
    CORS_ORIGINS: List[str] = [
        "http://localhost:5173",
        "http://localhost:3000",
    ]

    # 로봇 설정
    ROBOT_MODEL: str = "M0609"
    ROBOT_IP: str = "192.168.1.100"
    ROBOT_PORT: int = 12345

    # 개발 모드
    DEBUG: bool = True
    MOCK_ROBOT: bool = True

    # Firebase 설정
    FIREBASE_CREDENTIALS_PATH: str = "config/serviceAccountKey.json"
    FIREBASE_PROJECT_ID: str = "rokey-test-481307"

    class Config:
        env_file = ".env"
        case_sensitive = True


settings = Settings()
