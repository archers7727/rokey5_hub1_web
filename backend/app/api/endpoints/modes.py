"""
손질 모드 API 엔드포인트
"""
from fastapi import APIRouter, Query
from typing import Optional

router = APIRouter()

# Mock 데이터
MODES = [
    {
        "id": "frying",
        "name": "튀김 모드",
        "icon": "🍟",
        "description": "재료를 균등한 조각으로 나눕니다",
        "estimatedTime": 120,
        "supportedMaterials": ["onion", "potato"],
        "parameters": {
            "pieces": {
                "type": "select",
                "label": "조각 수",
                "options": [2, 4, 6, 8],
                "default": 4
            }
        }
    },
    {
        "id": "slicing",
        "name": "썰기 모드",
        "icon": "🔪",
        "description": "원하는 두께로 슬라이스합니다",
        "estimatedTime": 180,
        "supportedMaterials": ["onion", "potato"],
        "parameters": {
            "thickness": {
                "type": "slider",
                "label": "두께",
                "min": 5,
                "max": 30,
                "step": 5,
                "default": 10,
                "unit": "mm"
            }
        }
    }
]


@router.get("")
async def get_modes(material: Optional[str] = Query(None)):
    """모든 손질 모드 조회 (재료별 필터 가능)"""

    if material:
        # 특정 재료를 지원하는 모드만 필터링
        filtered_modes = [
            mode for mode in MODES
            if material in mode.get("supportedMaterials", [])
        ]
        return {
            "success": True,
            "data": filtered_modes
        }

    return {
        "success": True,
        "data": MODES
    }


@router.get("/{mode_id}")
async def get_mode(mode_id: str):
    """특정 모드 조회"""
    mode = next((m for m in MODES if m["id"] == mode_id), None)

    if not mode:
        return {
            "success": False,
            "error": "Mode not found"
        }

    return {
        "success": True,
        "data": mode
    }
