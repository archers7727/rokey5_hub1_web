"""
대시보드 API 엔드포인트
"""
from fastapi import APIRouter
from datetime import datetime, timedelta
import random

router = APIRouter()


@router.get("")
async def get_dashboard_data():
    """대시보드 데이터 조회"""

    # Mock 데이터
    today_stats = {
        "jobCount": 5,
        "totalTime": 3600,  # seconds
        "successRate": 100.0
    }

    recent_jobs = [
        {
            "id": "job-001",
            "material": "양파",
            "mode": "썰기",
            "completedAt": (datetime.now() - timedelta(minutes=10)).isoformat(),
            "duration": 180,
            "status": "completed"
        },
        {
            "id": "job-002",
            "material": "감자",
            "mode": "튀김",
            "completedAt": (datetime.now() - timedelta(hours=1)).isoformat(),
            "duration": 240,
            "status": "completed"
        },
        {
            "id": "job-003",
            "material": "양파",
            "mode": "튀김",
            "completedAt": (datetime.now() - timedelta(hours=2)).isoformat(),
            "duration": 200,
            "status": "completed"
        }
    ]

    return {
        "success": True,
        "data": {
            "todayStats": today_stats,
            "recentJobs": recent_jobs
        }
    }
