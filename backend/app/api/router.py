"""
API 라우터 통합
"""
from fastapi import APIRouter

from app.api.endpoints import (
    materials,
    modes,
    jobs,
    robot,
    dashboard,
    paths,
)

api_router = APIRouter()

# 각 엔드포인트 라우터 등록
api_router.include_router(
    materials.router,
    prefix="/materials",
    tags=["materials"]
)

api_router.include_router(
    modes.router,
    prefix="/modes",
    tags=["modes"]
)

api_router.include_router(
    jobs.router,
    prefix="/jobs",
    tags=["jobs"]
)

api_router.include_router(
    robot.router,
    prefix="/robot",
    tags=["robot"]
)

api_router.include_router(
    dashboard.router,
    prefix="/dashboard",
    tags=["dashboard"]
)

api_router.include_router(
    paths.router,
    prefix="/paths",
    tags=["paths"]
)
