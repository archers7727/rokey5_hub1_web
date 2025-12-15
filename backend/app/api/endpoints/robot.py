"""
로봇 상태 및 제어 API 엔드포인트
"""
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio
from typing import Dict
import random

router = APIRouter()

# 로봇 상태 (메모리)
robot_state = {
    "status": "idle",  # idle, working, emergency_stop, user_stop, error
    "model": "M1013",
    "isConnected": True,
    "jointAngles": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    "jointLoads": [0, 0, 0, 0, 0, 0],
    "tcpPosition": {
        "x": 0.450,
        "y": 0.000,
        "z": 0.680,
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0
    },
    "speed": 0.0
}


@router.get("/status")
async def get_robot_status():
    """로봇 현재 상태 조회"""
    return {
        "success": True,
        "data": robot_state
    }


@router.post("/emergency-stop")
async def emergency_stop():
    """비상 정지"""
    robot_state["status"] = "emergency_stop"
    robot_state["speed"] = 0.0

    return {
        "success": True,
        "message": "Emergency stop activated",
        "data": robot_state
    }


@router.post("/release-emergency")
async def release_emergency():
    """비상 정지 해제"""
    if robot_state["status"] == "emergency_stop":
        robot_state["status"] = "idle"

    return {
        "success": True,
        "message": "Emergency stop released",
        "data": robot_state
    }


@router.post("/resume")
async def resume_robot():
    """작업 재개"""
    if robot_state["status"] == "user_stop":
        robot_state["status"] = "working"

    return {
        "success": True,
        "message": "Robot resumed",
        "data": robot_state
    }


@router.post("/stop")
async def stop_robot():
    """완전 정지"""
    robot_state["status"] = "idle"
    robot_state["speed"] = 0.0

    return {
        "success": True,
        "message": "Robot stopped",
        "data": robot_state
    }


# WebSocket 연결 관리
robot_ws_connections: list[WebSocket] = []


@router.websocket("/ws/status")
async def robot_status_websocket(websocket: WebSocket):
    """로봇 상태 실시간 모니터링 WebSocket"""
    await websocket.accept()
    robot_ws_connections.append(websocket)

    try:
        while True:
            # 로봇 상태를 주기적으로 전송 (실제로는 로봇에서 데이터를 받아야 함)

            # Mock: 작동 중일 때 약간의 변화 추가
            if robot_state["status"] == "working":
                robot_state["jointAngles"] = [
                    angle + random.uniform(-0.5, 0.5)
                    for angle in robot_state["jointAngles"]
                ]
                robot_state["speed"] = random.uniform(0.1, 0.2)
            else:
                robot_state["speed"] = 0.0

            await websocket.send_json({
                "status": robot_state["status"],
                "model": robot_state["model"],
                "isConnected": robot_state["isConnected"],
                "jointAngles": robot_state["jointAngles"],
                "jointLoads": robot_state["jointLoads"],
                "tcpPosition": robot_state["tcpPosition"],
                "speed": robot_state["speed"]
            })

            await asyncio.sleep(0.5)  # 0.5초마다 업데이트

    except WebSocketDisconnect:
        robot_ws_connections.remove(websocket)
    except Exception as e:
        print(f"Robot WebSocket error: {e}")
        if websocket in robot_ws_connections:
            robot_ws_connections.remove(websocket)
