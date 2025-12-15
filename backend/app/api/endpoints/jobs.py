"""
작업 관리 API 엔드포인트
"""
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from typing import Dict, Any
from datetime import datetime
import asyncio
import uuid

router = APIRouter()

# 메모리 상의 작업 저장소 (향후 DB로 대체)
jobs_db: Dict[str, Any] = {}


@router.post("")
async def create_job(job_data: dict):
    """새 작업 생성"""
    job_id = str(uuid.uuid4())

    job = {
        "id": job_id,
        "material": job_data.get("material"),
        "mode": job_data.get("mode"),
        "parameters": job_data.get("parameters"),
        "status": "pending",
        "progress": 0,
        "currentStep": 0,
        "createdAt": datetime.now().isoformat(),
        "startedAt": None,
        "completedAt": None,
        "estimatedTime": job_data.get("estimatedTime", 0),
        "actualTime": 0,
    }

    jobs_db[job_id] = job

    return {
        "success": True,
        "data": job
    }


@router.get("/{job_id}")
async def get_job(job_id: str):
    """작업 조회"""
    job = jobs_db.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    return {
        "success": True,
        "data": job
    }


@router.get("")
async def get_jobs(limit: int = 10):
    """모든 작업 조회"""
    jobs_list = list(jobs_db.values())
    # 최신순 정렬
    jobs_list.sort(key=lambda x: x["createdAt"], reverse=True)

    return {
        "success": True,
        "data": jobs_list[:limit]
    }


@router.post("/{job_id}/start")
async def start_job(job_id: str):
    """작업 시작"""
    job = jobs_db.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    job["status"] = "running"
    job["startedAt"] = datetime.now().isoformat()

    return {
        "success": True,
        "data": job
    }


@router.post("/{job_id}/pause")
async def pause_job(job_id: str):
    """작업 일시정지"""
    job = jobs_db.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    job["status"] = "paused"

    return {
        "success": True,
        "data": job
    }


@router.post("/{job_id}/resume")
async def resume_job(job_id: str):
    """작업 재개"""
    job = jobs_db.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    job["status"] = "running"

    return {
        "success": True,
        "data": job
    }


@router.post("/{job_id}/stop")
async def stop_job(job_id: str):
    """작업 정지"""
    job = jobs_db.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    job["status"] = "stopped"
    job["completedAt"] = datetime.now().isoformat()

    return {
        "success": True,
        "data": job
    }


@router.post("/{job_id}/speed")
async def set_job_speed(job_id: str, speed_data: dict):
    """작업 속도 변경"""
    job = jobs_db.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    speed = speed_data.get("speed", 100)
    job["speed"] = speed

    return {
        "success": True,
        "data": job
    }


# WebSocket 연결 관리
active_connections: Dict[str, WebSocket] = {}


@router.websocket("/ws/{job_id}")
async def job_websocket(websocket: WebSocket, job_id: str):
    """작업 실시간 모니터링 WebSocket"""
    await websocket.accept()
    active_connections[job_id] = websocket

    try:
        job = jobs_db.get(job_id)

        if not job:
            await websocket.send_json({
                "error": "Job not found"
            })
            await websocket.close()
            return

        # 작업 진행 시뮬레이션
        total_steps = 10

        for step in range(total_steps):
            if job.get("status") != "running":
                # 일시정지 또는 정지 상태
                await asyncio.sleep(1)
                continue

            progress = (step + 1) / total_steps * 100

            # 작업 상태 업데이트
            job["progress"] = progress
            job["currentStep"] = step + 1

            # 클라이언트에 전송
            await websocket.send_json({
                "jobId": job_id,
                "status": job["status"],
                "progress": progress,
                "currentStep": step + 1,
                "totalSteps": total_steps,
                "stepDescription": f"단계 {step + 1}/{total_steps} 진행 중",
                "jointAngles": [45.2, -12.5, 60.8, 0.0, 90.0, 0.0],
                "tcpPosition": {
                    "x": 0.450,
                    "y": -0.120,
                    "z": 0.680
                },
                "speed": 0.15
            })

            await asyncio.sleep(2)  # 2초마다 업데이트

        # 작업 완료
        job["status"] = "completed"
        job["progress"] = 100
        job["completedAt"] = datetime.now().isoformat()

        await websocket.send_json({
            "jobId": job_id,
            "status": "completed",
            "progress": 100
        })

    except WebSocketDisconnect:
        if job_id in active_connections:
            del active_connections[job_id]
    except Exception as e:
        print(f"WebSocket error: {e}")
        await websocket.close()
