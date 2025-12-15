"""
Firestore 초기 데이터 삽입 스크립트
"""
import sys
import os

# 부모 디렉토리를 path에 추가
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from app.core.firestore import get_db, get_materials_ref, get_modes_ref


# 재료 데이터
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

# 가공 모드 데이터
MODES = [
    {
        "id": "frying",
        "name": "튀김",
        "emoji": "🍟",
        "description": "재료를 튀김용으로 자릅니다",
        "compatible_materials": ["onion", "potato"],
        "parameters": {
            "size": {
                "type": "select",
                "label": "크기",
                "options": ["small", "medium", "large"],
                "default": "medium"
            },
            "pieces": {
                "type": "select",
                "label": "조각 수",
                "options": [4, 6, 8, 12],
                "default": 8
            },
            "repeat_count": {
                "type": "number",
                "label": "반복 횟수",
                "min": 1,
                "max": 10,
                "default": 1
            }
        },
        "base_time": 30,
        "time_per_cut": 5
    },
    {
        "id": "slicing",
        "name": "썰기",
        "emoji": "🔪",
        "description": "재료를 얇게 썰어냅니다",
        "compatible_materials": ["onion", "potato"],
        "parameters": {
            "size": {
                "type": "select",
                "label": "크기",
                "options": ["small", "medium", "large"],
                "default": "medium"
            },
            "thickness": {
                "type": "number",
                "label": "두께 (mm)",
                "min": 1,
                "max": 10,
                "default": 3
            },
            "repeat_count": {
                "type": "number",
                "label": "반복 횟수",
                "min": 1,
                "max": 10,
                "default": 1
            }
        },
        "base_time": 40,
        "time_per_cut": 3
    }
]


def init_materials():
    """재료 데이터 초기화"""
    materials_ref = get_materials_ref()

    if materials_ref is None:
        print("❌ Firestore not available")
        return False

    print("📦 Initializing materials...")

    for material in MATERIALS:
        material_id = material.pop("id")
        materials_ref.document(material_id).set(material)
        print(f"  ✅ Added: {material['name']} ({material_id})")

    print(f"✅ Materials initialized ({len(MATERIALS)} items)")
    return True


def init_modes():
    """가공 모드 데이터 초기화"""
    modes_ref = get_modes_ref()

    if modes_ref is None:
        print("❌ Firestore not available")
        return False

    print("🔧 Initializing modes...")

    for mode in MODES:
        mode_id = mode.pop("id")
        modes_ref.document(mode_id).set(mode)
        print(f"  ✅ Added: {mode['name']} ({mode_id})")

    print(f"✅ Modes initialized ({len(MODES)} items)")
    return True


def init_robot_state():
    """로봇 상태 초기화"""
    db = get_db()

    if db is None:
        print("❌ Firestore not available")
        return False

    print("🤖 Initializing robot state...")

    initial_state = {
        "status": "idle",
        "current_task_id": None,
        "joint_states": {
            "position": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
            "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        },
        "tcp_position": {
            "x": 500.0,
            "y": 0.0,
            "z": 300.0,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0
        },
        "error_state": None
    }

    db.collection('robot_state').document('current').set(initial_state)
    print("✅ Robot state initialized")
    return True


def main():
    """메인 함수"""
    print("\n" + "=" * 50)
    print("🚀 Firestore 초기 데이터 삽입")
    print("=" * 50 + "\n")

    # Firestore 초기화
    db = get_db()
    if db is None:
        print("\n❌ Firestore initialization failed!")
        print("⚠️  Please check:")
        print("   1. FIREBASE_CREDENTIALS_PATH in .env")
        print("   2. serviceAccountKey.json file exists")
        print("   3. Firebase project ID is correct")
        sys.exit(1)

    # 데이터 삽입
    success = True
    success &= init_materials()
    success &= init_modes()
    success &= init_robot_state()

    if success:
        print("\n" + "=" * 50)
        print("✅ 모든 데이터가 성공적으로 초기화되었습니다!")
        print("=" * 50 + "\n")
    else:
        print("\n❌ 일부 데이터 초기화에 실패했습니다.")
        sys.exit(1)


if __name__ == "__main__":
    main()
