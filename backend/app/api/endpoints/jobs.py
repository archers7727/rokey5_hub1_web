"""
작업 관리 API 엔드포인트
"""
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from typing import Dict, Any
from datetime import datetime
from google.cloud import firestore as firestore_lib
import asyncio
import uuid

from app.core.firestore import get_db, get_tasks_ref, get_jobs_ref

router = APIRouter()


@router.post("")
async def create_job(job_data: dict):
    """새 작업 생성 → Firestore tasks/ 컬렉션에 추가"""
    tasks_ref = get_tasks_ref()

    # Firestore 사용 불가능하면 메모리 저장
    if tasks_ref is None:
        job_id = str(uuid.uuid4())
        job = {
            "id": job_id,
            "material": job_data.get("material"),
            "mode": job_data.get("mode"),
            "parameters": job_data.get("parameters"),
            "status": "pending",
            "progress": 0,
            "createdAt": datetime.now().isoformat(),
            "estimatedTime": job_data.get("estimatedTime", 0),
        }
        return {"success": True, "data": job}

    # Firestore에 task 생성
    task_doc = {
        "material": job_data.get("material"),
        "mode": job_data.get("mode"),
        "parameters": job_data.get("parameters"),
        "status": "pending",
        "priority": 1,
        "progress": 0,
        "current_step": None,
        "created_at": firestore_lib.SERVER_TIMESTAMP,
        "started_at": None,
        "completed_at": None,
        "estimated_time": job_data.get("estimatedTime", 0),
    }

    # Firestore에 추가
    task_ref = tasks_ref.document()
    task_ref.set(task_doc)

    # 생성된 task 데이터 반환
    task_data = task_ref.get().to_dict()
    task_data["id"] = task_ref.id

    return {
        "success": True,
        "data": task_data,
        "task_id": task_ref.id,
        "message": "Task queued successfully"
    }


@router.get("/{job_id}")
async def get_job(job_id: str):
    """작업 조회 (tasks + jobs 컬렉션 검색)"""
    tasks_ref = get_tasks_ref()
    jobs_ref = get_jobs_ref()

    if tasks_ref is None or jobs_ref is None:
        raise HTTPException(status_code=503, detail="Firestore not available")

    # tasks에서 먼저 검색
    task_doc = tasks_ref.document(job_id).get()

    if task_doc.exists:
        task_data = task_doc.to_dict()
        task_data["id"] = task_doc.id
        return {"success": True, "data": task_data}

    # jobs 히스토리에서 검색
    job_doc = jobs_ref.document(job_id).get()

    if job_doc.exists:
        job_data = job_doc.to_dict()
        job_data["id"] = job_doc.id
        return {"success": True, "data": job_data}

    raise HTTPException(status_code=404, detail="Job not found")


@router.get("")
async def get_jobs(limit: int = 10):
    """모든 작업 조회 (tasks + jobs 합쳐서 반환)"""
    tasks_ref = get_tasks_ref()
    jobs_ref = get_jobs_ref()

    if tasks_ref is None or jobs_ref is None:
        return {"success": True, "data": []}

    all_jobs = []

    # 진행 중인 tasks 가져오기
    tasks_query = tasks_ref.order_by("created_at", direction=firestore_lib.Query.DESCENDING).limit(limit)
    for doc in tasks_query.stream():
        task_data = doc.to_dict()
        task_data["id"] = doc.id
        all_jobs.append(task_data)

    # 완료된 jobs 가져오기
    jobs_query = jobs_ref.order_by("created_at", direction=firestore_lib.Query.DESCENDING).limit(limit)
    for doc in jobs_query.stream():
        job_data = doc.to_dict()
        job_data["id"] = doc.id
        all_jobs.append(job_data)

    # 최신순 정렬
    all_jobs.sort(key=lambda x: x.get("created_at", datetime.min), reverse=True)

    return {
        "success": True,
        "data": all_jobs[:limit]
    }


@router.post("/{job_id}/start")
async def start_job(job_id: str):
    """작업 시작 (status 업데이트)"""
    tasks_ref = get_tasks_ref()

    if tasks_ref is None:
        raise HTTPException(status_code=503, detail="Firestore not available")

    task_ref = tasks_ref.document(job_id)
    task_doc = task_ref.get()

    if not task_doc.exists:
        raise HTTPException(status_code=404, detail="Job not found")

    # 상태 업데이트
    task_ref.update({
        "status": "executing",
        "started_at": firestore_lib.SERVER_TIMESTAMP
    })

    # 업데이트된 데이터 반환
    updated_task = task_ref.get().to_dict()
    updated_task["id"] = job_id

    return {
        "success": True,
        "data": updated_task
    }


@router.post("/{job_id}/pause")
async def pause_job(job_id: str):
    """작업 일시정지"""
    tasks_ref = get_tasks_ref()

    if tasks_ref is None:
        raise HTTPException(status_code=503, detail="Firestore not available")

    task_ref = tasks_ref.document(job_id)
    task_doc = task_ref.get()

    if not task_doc.exists:
        raise HTTPException(status_code=404, detail="Job not found")

    task_ref.update({"status": "paused"})

    updated_task = task_ref.get().to_dict()
    updated_task["id"] = job_id

    return {
        "success": True,
        "data": updated_task
    }


@router.post("/{job_id}/resume")
async def resume_job(job_id: str):
    """작업 재개"""
    tasks_ref = get_tasks_ref()

    if tasks_ref is None:
        raise HTTPException(status_code=503, detail="Firestore not available")

    task_ref = tasks_ref.document(job_id)
    task_doc = task_ref.get()

    if not task_doc.exists:
        raise HTTPException(status_code=404, detail="Job not found")

    task_ref.update({"status": "executing"})

    updated_task = task_ref.get().to_dict()
    updated_task["id"] = job_id

    return {
        "success": True,
        "data": updated_task
    }


@router.post("/{job_id}/stop")
async def stop_job(job_id: str):
    """작업 정지 → jobs 컬렉션으로 이동"""
    tasks_ref = get_tasks_ref()
    jobs_ref = get_jobs_ref()

    if tasks_ref is None or jobs_ref is None:
        raise HTTPException(status_code=503, detail="Firestore not available")

    task_ref = tasks_ref.document(job_id)
    task_doc = task_ref.get()

    if not task_doc.exists:
        raise HTTPException(status_code=404, detail="Job not found")

    task_data = task_doc.to_dict()

    # jobs 컬렉션에 저장
    job_data = {
        **task_data,
        "status": "stopped",
        "completed_at": firestore_lib.SERVER_TIMESTAMP
    }

    jobs_ref.document(job_id).set(job_data)

    # tasks에서 삭제
    task_ref.delete()

    job_data["id"] = job_id

    return {
        "success": True,
        "data": job_data
    }


@router.post("/{job_id}/speed")
async def set_job_speed(job_id: str, speed_data: dict):
    """작업 속도 변경"""
    tasks_ref = get_tasks_ref()

    if tasks_ref is None:
        raise HTTPException(status_code=503, detail="Firestore not available")

    task_ref = tasks_ref.document(job_id)
    task_doc = task_ref.get()

    if not task_doc.exists:
        raise HTTPException(status_code=404, detail="Job not found")

    speed = speed_data.get("speed", 100)
    task_ref.update({"speed": speed})

    updated_task = task_ref.get().to_dict()
    updated_task["id"] = job_id

    return {
        "success": True,
        "data": updated_task
    }


# WebSocket 연결 관리
active_connections: Dict[str, WebSocket] = {}


@router.websocket("/ws/{job_id}")
async def job_websocket(websocket: WebSocket, job_id: str):
    """작업 실시간 모니터링 WebSocket (Firestore 연동)"""
    await websocket.accept()
    active_connections[job_id] = websocket

    try:
        tasks_ref = get_tasks_ref()
        jobs_ref = get_jobs_ref()

        if tasks_ref is None:
            await websocket.send_json({"error": "Firestore not available"})
            await websocket.close()
            return

        task_ref = tasks_ref.document(job_id)
        task_doc = task_ref.get()

        if not task_doc.exists:
            await websocket.send_json({"error": "Job not found"})
            await websocket.close()
            return

        # 작업 진행 시뮬레이션
        total_steps = 10

        for step in range(total_steps):
            # Firestore에서 최신 상태 가져오기
            task_doc = task_ref.get()

            if not task_doc.exists:
                break

            task_data = task_doc.to_dict()
            status = task_data.get("status", "pending")

            if status != "executing":
                # 일시정지 또는 정지 상태
                await asyncio.sleep(1)
                continue

            progress = (step + 1) / total_steps * 100

            # Firestore 상태 업데이트
            task_ref.update({
                "progress": progress,
                "current_step": step + 1
            })

            # 클라이언트에 전송
            await websocket.send_json({
                "jobId": job_id,
                "status": status,
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

        # 작업 완료 → jobs 컬렉션으로 이동
        task_data = task_ref.get().to_dict()

        job_data = {
            **task_data,
            "status": "completed",
            "progress": 100,
            "completed_at": firestore_lib.SERVER_TIMESTAMP
        }

        if jobs_ref:
            jobs_ref.document(job_id).set(job_data)
            task_ref.delete()

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
