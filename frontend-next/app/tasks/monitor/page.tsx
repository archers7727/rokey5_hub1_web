'use client'

import { useState, useEffect } from 'react'
import { useRouter } from 'next/navigation'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { RobotArm3DViewer } from '@/components/RobotArm3DViewer'
import { useTasksRealtime } from '@/hooks/useTasksRealtime'
import { useRobotStateRealtime } from '@/hooks/useRobotStateRealtime'

export const dynamic = 'force-dynamic'

interface Task {
  id: string
  material_id: string
  mode_id: string
  parameters: any
  status: string
  progress: number
  priority: number
  created_at: string
  updated_at?: string
  estimated_time?: number
}

export default function TaskMonitor() {
  const router = useRouter()
  const tasks = useTasksRealtime()
  const robotState = useRobotStateRealtime()
  const [materials, setMaterials] = useState<any>({})
  const [modes, setModes] = useState<any>({})

  useEffect(() => {
    loadMaterialsAndModes()
  }, [])

  const loadMaterialsAndModes = async () => {
    try {
      // Load materials
      const materialsRes = await fetch('/api/materials')
      const materialsData = await materialsRes.json()
      if (materialsData.success) {
        const materialsMap: any = {}
        materialsData.data.forEach((m: any) => {
          materialsMap[m.id] = m
        })
        setMaterials(materialsMap)
      }

      // Load modes
      const modesRes = await fetch('/api/modes')
      const modesData = await modesRes.json()
      if (modesData.success) {
        const modesMap: any = {}
        modesData.data.forEach((m: any) => {
          modesMap[m.id] = m
        })
        setModes(modesMap)
      }
    } catch (error) {
      console.error('Failed to load data:', error)
    }
  }

  const activeTasks = tasks.filter((t: Task) =>
    ['pending', 'queued', 'running'].includes(t.status)
  )

  const completedTasks = tasks.filter((t: Task) =>
    ['completed', 'failed', 'cancelled'].includes(t.status)
  )

  const getStatusColor = (status: string) => {
    const colors: Record<string, string> = {
      pending: 'bg-gray-500',
      queued: 'bg-blue-500',
      running: 'bg-green-500',
      paused: 'bg-yellow-500',
      completed: 'bg-gray-400',
      failed: 'bg-red-500',
      cancelled: 'bg-gray-400',
    }
    return colors[status] || 'bg-gray-500'
  }

  const getStatusText = (status: string) => {
    const texts: Record<string, string> = {
      pending: '대기 중',
      queued: '준비 중',
      running: '진행 중',
      paused: '일시 정지',
      completed: '완료',
      failed: '실패',
      cancelled: '취소됨',
    }
    return texts[status] || status
  }

  const formatTime = (seconds: number) => {
    if (seconds < 60) return `${seconds}초`
    const mins = Math.floor(seconds / 60)
    const secs = seconds % 60
    return secs > 0 ? `${mins}분 ${secs}초` : `${mins}분`
  }

  const calculateDuration = (task: Task): number => {
    // created_at(시작) ~ updated_at(종료) 사이의 시간 계산
    if (!task.created_at) return 0

    const start = new Date(task.created_at).getTime()
    // 완료된 작업은 updated_at, 진행 중이면 현재 시간 사용
    const end = task.updated_at
      ? new Date(task.updated_at).getTime()
      : Date.now()
    const diffMs = end - start

    // 밀리초를 초로 변환
    return Math.floor(diffMs / 1000)
  }

  const renderTaskCard = (task: Task) => {
    const material = materials[task.material_id]
    const mode = modes[task.mode_id]

    if (!material || !mode) return null

    return (
      <Card key={task.id} variant="elevated">
        <div className="space-y-4">
          {/* Header */}
          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-3">
              <span className="text-3xl">{material.emoji}</span>
              <span className="text-2xl">{mode.icon}</span>
              <div>
                <h3 className="text-lg font-semibold text-gray-900">
                  {material.name} - {mode.name}
                </h3>
                <p className="text-sm text-gray-600">{mode.description}</p>
              </div>
            </div>
            <div className="flex items-center space-x-2">
              <div className={`w-3 h-3 rounded-full ${getStatusColor(task.status)}`} />
              <span className="text-sm font-medium text-gray-700">
                {getStatusText(task.status)}
              </span>
            </div>
          </div>

          {/* Parameters */}
          {task.parameters && Object.keys(task.parameters).length > 0 && (
            <div className="bg-gray-50 rounded-lg p-3">
              <div className="text-xs font-semibold text-gray-500 uppercase mb-2">
                작업 설정
              </div>
              <div className="space-y-1">
                {task.parameters.thickness && (
                  <div className="flex items-center justify-between text-sm">
                    <span className="text-gray-600">두께</span>
                    <span className="font-medium text-gray-900">
                      {task.parameters.thickness}mm
                    </span>
                  </div>
                )}
              </div>
            </div>
          )}

          {/* Progress Bar */}
          {task.status === 'running' && (
            <div className="space-y-2">
              <div className="flex items-center justify-between text-sm">
                <span className="text-gray-600">진행률</span>
                <span className="font-semibold text-gray-900">{task.progress}%</span>
              </div>
              <div className="w-full bg-gray-200 rounded-full h-2.5">
                <div
                  className="bg-green-600 h-2.5 rounded-full transition-all duration-300"
                  style={{ width: `${task.progress}%` }}
                />
              </div>
            </div>
          )}

          {/* Time Info */}
          <div className="flex items-center justify-between text-sm text-gray-500">
            <div>
              우선순위: {task.priority}
            </div>
            <div>
              {task.status === 'completed' ? '소요 시간' : '경과 시간'}: {formatTime(calculateDuration(task))}
            </div>
          </div>

          {/* Task ID */}
          <div className="text-xs text-gray-400 font-mono">
            ID: {task.id.substring(0, 8)}...
          </div>
        </div>
      </Card>
    )
  }

  // 현재 실행 중인 작업의 관절 각도
  const runningTask = activeTasks.find((t: Task) => t.status === 'running')
  const currentProgress = runningTask?.progress || 0

  // 로봇팔 애니메이션을 위한 상태
  const [animatedJointAngles, setAnimatedJointAngles] = useState<number[]>([0, 0, 0, 0, 0, 0])

  // 진행 중인 작업이 있으면 로봇팔 애니메이션
  useEffect(() => {
    if (!runningTask) {
      // 작업이 없으면 홈 포지션
      setAnimatedJointAngles([0, 0, 0, 0, 0, 0])
      return
    }

    // 작업 진행 중이면 관절을 부드럽게 움직임
    const interval = setInterval(() => {
      const time = Date.now() / 1000
      // 진행률이 0이어도 최소한의 움직임 보장 (0.3 ~ 1.0)
      const progress = Math.max(0.3, currentProgress / 100)

      // 작업 진행도에 따라 관절 각도 변경
      setAnimatedJointAngles([
        Math.sin(time * 0.5) * 30 * progress,           // Joint 1: 좌우 회전
        -20 + Math.sin(time * 0.3) * 15 * progress,     // Joint 2: 위아래
        90 + Math.cos(time * 0.4) * 20 * progress,      // Joint 3: 엘보우
        Math.sin(time * 0.6) * 25 * progress,           // Joint 4: 손목 회전
        45 + Math.cos(time * 0.5) * 15 * progress,      // Joint 5: 손목 벤드
        Math.sin(time * 0.7) * 30 * progress,           // Joint 6: 플랜지 회전
      ])
    }, 50) // 50ms마다 업데이트

    return () => clearInterval(interval)
  }, [runningTask, currentProgress])

  const jointAngles = robotState?.joint_angles || animatedJointAngles

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">작업 모니터</h1>
          <p className="text-sm text-gray-600 mt-1">
            실시간으로 작업 진행 상황을 확인하세요
          </p>
        </div>
        <Link href="/dashboard">
          <Button variant="ghost">← 대시보드</Button>
        </Link>
      </div>

      {/* 좌우 분할 레이아웃 */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* 좌측: 작업 목록 */}
        <div className="space-y-6">
          {/* Robot Status */}
          {robotState && (
            <Link href="/robot/status" className="block">
              <Card variant="outlined" className="bg-gray-50 cursor-pointer hover:shadow-md transition-shadow">
                <div className="flex items-center justify-between">
                  <div className="flex items-center space-x-3">
                    <div className={`w-4 h-4 rounded-full ${getStatusColor(robotState.status)}`} />
                    <div>
                      <div className="font-semibold text-gray-900">
                        로봇 상태: {getStatusText(robotState.status)}
                      </div>
                      <div className="text-sm text-gray-600">Doosan M0609 (클릭하여 상세 보기)</div>
                    </div>
                  </div>
                  {robotState.current_task_id && (
                    <div className="text-sm text-gray-600">
                      작업 ID: {robotState.current_task_id.substring(0, 8)}...
                    </div>
                  )}
                </div>
              </Card>
            </Link>
          )}

          {/* Active Tasks */}
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <h2 className="text-2xl font-bold text-gray-900">
                진행 중인 작업 ({activeTasks.length})
              </h2>
              <Link href="/job/material">
                <Button variant="primary">+ 새 작업 추가</Button>
              </Link>
            </div>

            {activeTasks.length > 0 ? (
              <div className="space-y-4">
                {activeTasks.map((task: Task) => renderTaskCard(task))}
              </div>
            ) : (
              <Card variant="outlined">
                <div className="text-center py-12">
                  <p className="text-gray-500 mb-4">진행 중인 작업이 없습니다</p>
                  <Link href="/job/material">
                    <Button variant="secondary">새 작업 시작하기</Button>
                  </Link>
                </div>
              </Card>
            )}
          </div>

          {/* Completed Tasks */}
          {completedTasks.length > 0 && (
            <div className="space-y-4">
              <h2 className="text-2xl font-bold text-gray-900">
                완료된 작업 ({completedTasks.length})
              </h2>
              <div className="space-y-4">
                {completedTasks.slice(0, 5).map((task: Task) => renderTaskCard(task))}
              </div>
            </div>
          )}
        </div>

        {/* 우측: 3D 로봇팔 뷰 */}
        <div className="lg:sticky lg:top-6 h-[600px] rounded-lg overflow-hidden shadow-2xl">
          <RobotArm3DViewer jointAngles={jointAngles} progress={currentProgress} />
        </div>
      </div>
    </div>
  )
}
