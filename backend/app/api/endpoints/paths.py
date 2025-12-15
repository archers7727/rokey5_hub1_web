"""
작업 경로 설정 API 엔드포인트
"""
from fastapi import APIRouter, HTTPException
from typing import Dict, Any
import uuid

router = APIRouter()

# 메모리 상의 경로 저장소
paths_db: Dict[str, Any] = {
    "default": {
        "id": "default",
        "name": "기본 경로",
        "isLocked": True,
        "points": [
            {
                "id": "p1",
                "type": "pickup",
                "x": 0.3,
                "y": 0.2,
                "z": 0.5,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0
            },
            {
                "id": "p2",
                "type": "work",
                "x": 0.4,
                "y": 0.0,
                "z": 0.6,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0
            },
            {
                "id": "p3",
                "type": "place",
                "x": 0.5,
                "y": -0.2,
                "z": 0.5,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0
            }
        ],
        "createdAt": "2025-12-15T00:00:00",
        "updatedAt": "2025-12-15T00:00:00"
    }
}


@router.get("")
async def get_paths():
    """모든 경로 조회"""
    return {
        "success": True,
        "data": list(paths_db.values())
    }


@router.get("/{path_id}")
async def get_path(path_id: str):
    """특정 경로 조회"""
    path = paths_db.get(path_id)

    if not path:
        raise HTTPException(status_code=404, detail="Path not found")

    return {
        "success": True,
        "data": path
    }


@router.post("")
async def create_path(path_data: dict):
    """새 경로 생성"""
    path_id = str(uuid.uuid4())

    path = {
        "id": path_id,
        "name": path_data.get("name", "새 경로"),
        "isLocked": False,
        "points": path_data.get("points", []),
        "createdAt": path_data.get("createdAt"),
        "updatedAt": path_data.get("updatedAt")
    }

    paths_db[path_id] = path

    return {
        "success": True,
        "data": path
    }


@router.put("/{path_id}")
async def update_path(path_id: str, path_data: dict):
    """경로 수정"""
    path = paths_db.get(path_id)

    if not path:
        raise HTTPException(status_code=404, detail="Path not found")

    if path.get("isLocked"):
        raise HTTPException(status_code=403, detail="Cannot modify locked path")

    # 업데이트
    path["name"] = path_data.get("name", path["name"])
    path["points"] = path_data.get("points", path["points"])
    path["updatedAt"] = path_data.get("updatedAt")

    return {
        "success": True,
        "data": path
    }


@router.delete("/{path_id}")
async def delete_path(path_id: str):
    """경로 삭제"""
    path = paths_db.get(path_id)

    if not path:
        raise HTTPException(status_code=404, detail="Path not found")

    if path.get("isLocked"):
        raise HTTPException(status_code=403, detail="Cannot delete locked path")

    del paths_db[path_id]

    return {
        "success": True,
        "message": "Path deleted"
    }


@router.post("/{path_id}/validate")
async def validate_path(path_id: str):
    """경로 검증"""
    path = paths_db.get(path_id)

    if not path:
        raise HTTPException(status_code=404, detail="Path not found")

    points = path.get("points", [])

    # 간단한 검증
    if len(points) < 2:
        return {
            "valid": False,
            "error": "최소 2개 이상의 포인트가 필요합니다"
        }

    # 실제로는 로봇 도달 가능 여부, 충돌 검사 등을 해야 함

    return {
        "valid": True,
        "message": "경로가 유효합니다"
    }
