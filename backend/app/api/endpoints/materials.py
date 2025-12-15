"""
재료 관리 API 엔드포인트
"""
from fastapi import APIRouter
from typing import List

from app.core.firestore import get_materials_ref

router = APIRouter()

# Mock 데이터 (Firestore 사용 불가능할 때 fallback)
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
    """모든 재료 목록 조회 (Firestore 우선, 없으면 Mock 데이터)"""
    materials_ref = get_materials_ref()

    # Firestore에서 조회
    if materials_ref is not None:
        materials = []
        docs = materials_ref.stream()

        for doc in docs:
            material_data = doc.to_dict()
            material_data["id"] = doc.id
            materials.append(material_data)

        if materials:
            return {
                "success": True,
                "data": materials
            }

    # Firestore 사용 불가 또는 데이터 없음 → Mock 데이터 반환
    return {
        "success": True,
        "data": MATERIALS
    }


@router.get("/{material_id}")
async def get_material(material_id: str):
    """특정 재료 조회"""
    materials_ref = get_materials_ref()

    # Firestore에서 조회
    if materials_ref is not None:
        doc = materials_ref.document(material_id).get()

        if doc.exists:
            material_data = doc.to_dict()
            material_data["id"] = doc.id
            return {
                "success": True,
                "data": material_data
            }

    # Firestore에 없으면 Mock 데이터에서 검색
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
