"""
재료 관리 API 엔드포인트
"""
from fastapi import APIRouter
from typing import List

router = APIRouter()

# Mock 데이터
MATERIALS = [
    {
        "id": "onion",
        "name": "양파",
        "emoji": "🧅",
        "description": "신선한 양파로 다양한 요리를",
        "category": "vegetable",
        "sizes": {
            "small": 6,
            "medium": 8,
            "large": 10
        }
    },
    {
        "id": "potato",
        "name": "감자",
        "emoji": "🥔",
        "description": "고소한 감자로 튀김 요리를",
        "category": "vegetable",
        "sizes": {
            "small": 5,
            "medium": 7,
            "large": 9
        }
    }
]


@router.get("")
async def get_materials():
    """모든 재료 목록 조회"""
    return {
        "success": True,
        "data": MATERIALS
    }


@router.get("/{material_id}")
async def get_material(material_id: str):
    """특정 재료 조회"""
    material = next((m for m in MATERIALS if m["id"] == material_id), None)

    if not material:
        return {
            "success": False,
            "error": "Material not found"
        }

    return {
        "success": True,
        "data": material
    }
