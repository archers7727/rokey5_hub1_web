'use client'

import { useState, useEffect } from 'react'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { useRobotStateRealtime } from '@/hooks/useRobotStateRealtime'

export const dynamic = 'force-dynamic'

export default function RobotStatus() {
  const robotState = useRobotStateRealtime()
  const [currentTask, setCurrentTask] = useState<any>(null)
  const [sendingCommand, setSendingCommand] = useState(false)

  useEffect(() => {
    if (robotState?.current_task_id) {
      loadCurrentTask(robotState.current_task_id)
    } else {
      setCurrentTask(null)
    }
  }, [robotState?.current_task_id])

  const loadCurrentTask = async (taskId: string) => {
    try {
      const response = await fetch(`/api/jobs?task_id=${taskId}`)
      const result = await response.json()
      if (result.success && result.data.length > 0) {
        setCurrentTask(result.data[0])
      }
    } catch (error) {
      console.error('Failed to load task:', error)
    }
  }

  const sendCommand = async (command: string) => {
    if (sendingCommand) return

    setSendingCommand(true)
    try {
      const response = await fetch('/api/robot/command', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ command }),
      })

      const result = await response.json()

      if (result.success) {
        alert(`명령 전송 성공: ${command}`)
      } else {
        alert(`명령 전송 실패: ${result.error}`)
      }
    } catch (error) {
      console.error('Failed to send command:', error)
      alert('명령 전송 중 오류가 발생했습니다.')
    } finally {
      setSendingCommand(false)
    }
  }

  const handlePause = () => {
    if (confirm('로봇을 일시정지하시겠습니까?')) {
      sendCommand('pause')
    }
  }

  const handleResume = () => {
    if (confirm('로봇을 재개하시겠습니까?')) {
      sendCommand('resume')
    }
  }

  const handleStop = () => {
    if (confirm('로봇을 정지하시겠습니까?')) {
      sendCommand('stop')
    }
  }

  const handleEmergencyStop = () => {
    if (confirm('⚠️ 긴급정지를 실행하시겠습니까? 이 작업은 즉시 로봇을 정지시킵니다.')) {
      sendCommand('emergency_stop')
    }
  }

  const getStatusColor = (status: string) => {
    const colors: Record<string, string> = {
      idle: 'bg-gray-500',
      running: 'bg-green-500',
      paused: 'bg-yellow-500',
      error: 'bg-red-500',
    }
    return colors[status] || 'bg-gray-500'
  }

  const getStatusText = (status: string) => {
    const texts: Record<string, string> = {
      idle: '대기 중',
      running: '작동 중',
      paused: '일시 정지',
      error: '에러',
    }
    return texts[status] || '알 수 없음'
  }

  const formatTimestamp = (timestamp: string) => {
    const date = new Date(timestamp)
    return date.toLocaleString('ko-KR', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
    })
  }

  const formatPosition = (value: number) => {
    return value.toFixed(2)
  }

  if (!robotState) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <div className="text-center">
          <div className="text-lg text-gray-600 mb-4">로봇 상태 로딩 중...</div>
          <div className="text-sm text-gray-500">Supabase 연결 확인 중</div>
        </div>
      </div>
    )
  }

  const jointStates = robotState.joint_states || {
    position: [0, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0],
    effort: [0, 0, 0, 0, 0, 0],
  }

  const tcpPosition = robotState.tcp_position || {
    x: 0,
    y: 0,
    z: 0,
    rx: 0,
    ry: 0,
    rz: 0,
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">로봇 상태 모니터</h1>
          <p className="text-sm text-gray-600 mt-1">
            Doosan M0609 실시간 상태 확인
          </p>
        </div>
        <Link href="/dashboard">
          <Button variant="ghost">← 대시보드</Button>
        </Link>
      </div>

      {/* Robot Status Overview */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        {/* Status */}
        <Card variant="elevated">
          <div className="space-y-2">
            <div className="text-sm font-semibold text-gray-500 uppercase">
              상태
            </div>
            <div className="flex items-center space-x-2">
              <div className={`w-4 h-4 rounded-full ${getStatusColor(robotState.status)}`} />
              <span className="text-xl font-bold text-gray-900">
                {getStatusText(robotState.status)}
              </span>
            </div>
          </div>
        </Card>

        {/* Current Task */}
        <Card variant="elevated">
          <div className="space-y-2">
            <div className="text-sm font-semibold text-gray-500 uppercase">
              현재 작업
            </div>
            <div className="text-lg font-semibold text-gray-900">
              {robotState.current_task_id ? (
                <span className="font-mono text-sm">
                  {robotState.current_task_id.substring(0, 8)}...
                </span>
              ) : (
                <span className="text-gray-400">없음</span>
              )}
            </div>
          </div>
        </Card>

        {/* Error State */}
        <Card variant="elevated">
          <div className="space-y-2">
            <div className="text-sm font-semibold text-gray-500 uppercase">
              에러 상태
            </div>
            <div className="text-lg font-semibold">
              {robotState.error_state ? (
                <span className="text-red-600">{robotState.error_state}</span>
              ) : (
                <span className="text-green-600">정상</span>
              )}
            </div>
          </div>
        </Card>

        {/* Last Update */}
        <Card variant="elevated">
          <div className="space-y-2">
            <div className="text-sm font-semibold text-gray-500 uppercase">
              마지막 업데이트
            </div>
            <div className="text-sm font-medium text-gray-900">
              {robotState.updated_at ? formatTimestamp(robotState.updated_at) : '-'}
            </div>
          </div>
        </Card>
      </div>

      {/* Joint States */}
      <Card variant="elevated">
        <div className="space-y-4">
          <h2 className="text-xl font-bold text-gray-900">관절 상태 (Joint States)</h2>
          <p className="text-sm text-gray-600">6축 로봇 관절의 위치, 속도, 토크 정보</p>

          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="border-b border-gray-200">
                  <th className="text-left py-3 px-4 font-semibold text-gray-700">
                    관절
                  </th>
                  <th className="text-right py-3 px-4 font-semibold text-gray-700">
                    위치 (deg)
                  </th>
                  <th className="text-right py-3 px-4 font-semibold text-gray-700">
                    속도 (deg/s)
                  </th>
                  <th className="text-right py-3 px-4 font-semibold text-gray-700">
                    토크 (Nm)
                  </th>
                  <th className="text-center py-3 px-4 font-semibold text-gray-700">
                    상태
                  </th>
                </tr>
              </thead>
              <tbody>
                {[0, 1, 2, 3, 4, 5].map((index) => {
                  const position = jointStates.position[index] || 0
                  const velocity = jointStates.velocity[index] || 0
                  const effort = jointStates.effort[index] || 0
                  const isMoving = Math.abs(velocity) > 0.01

                  return (
                    <tr key={index} className="border-b border-gray-100 hover:bg-gray-50">
                      <td className="py-3 px-4 font-medium text-gray-900">
                        Joint {index + 1}
                      </td>
                      <td className="py-3 px-4 text-right font-mono text-gray-900">
                        {formatPosition(position)}°
                      </td>
                      <td className="py-3 px-4 text-right font-mono text-gray-900">
                        {formatPosition(velocity)}
                      </td>
                      <td className="py-3 px-4 text-right font-mono text-gray-900">
                        {formatPosition(effort)}
                      </td>
                      <td className="py-3 px-4 text-center">
                        {isMoving ? (
                          <span className="inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-green-100 text-green-800">
                            움직임
                          </span>
                        ) : (
                          <span className="inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
                            정지
                          </span>
                        )}
                      </td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
          </div>
        </div>
      </Card>

      {/* TCP Position */}
      <Card variant="elevated">
        <div className="space-y-4">
          <h2 className="text-xl font-bold text-gray-900">
            TCP 위치 (Tool Center Point)
          </h2>
          <p className="text-sm text-gray-600">로봇 엔드 이펙터의 위치 및 자세</p>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            {/* Position */}
            <div>
              <h3 className="text-sm font-semibold text-gray-500 uppercase mb-3">
                위치 (Position)
              </h3>
              <div className="space-y-2">
                <div className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                  <span className="text-sm font-medium text-gray-700">X</span>
                  <span className="text-lg font-mono font-semibold text-gray-900">
                    {formatPosition(tcpPosition.x)} mm
                  </span>
                </div>
                <div className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                  <span className="text-sm font-medium text-gray-700">Y</span>
                  <span className="text-lg font-mono font-semibold text-gray-900">
                    {formatPosition(tcpPosition.y)} mm
                  </span>
                </div>
                <div className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                  <span className="text-sm font-medium text-gray-700">Z</span>
                  <span className="text-lg font-mono font-semibold text-gray-900">
                    {formatPosition(tcpPosition.z)} mm
                  </span>
                </div>
              </div>
            </div>

            {/* Orientation */}
            <div>
              <h3 className="text-sm font-semibold text-gray-500 uppercase mb-3">
                자세 (Orientation)
              </h3>
              <div className="space-y-2">
                <div className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                  <span className="text-sm font-medium text-gray-700">RX (Roll)</span>
                  <span className="text-lg font-mono font-semibold text-gray-900">
                    {formatPosition(tcpPosition.rx)}°
                  </span>
                </div>
                <div className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                  <span className="text-sm font-medium text-gray-700">RY (Pitch)</span>
                  <span className="text-lg font-mono font-semibold text-gray-900">
                    {formatPosition(tcpPosition.ry)}°
                  </span>
                </div>
                <div className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                  <span className="text-sm font-medium text-gray-700">RZ (Yaw)</span>
                  <span className="text-lg font-mono font-semibold text-gray-900">
                    {formatPosition(tcpPosition.rz)}°
                  </span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </Card>

      {/* Visual Indicators */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Joint Position Bars */}
        <Card variant="elevated">
          <div className="space-y-4">
            <h3 className="text-lg font-semibold text-gray-900">관절 위치 시각화</h3>
            <div className="space-y-3">
              {[0, 1, 2, 3, 4, 5].map((index) => {
                const position = jointStates.position[index] || 0
                // 관절 범위를 -180 ~ 180으로 가정하고 0 ~ 100%로 정규화
                const normalized = ((position + 180) / 360) * 100

                return (
                  <div key={index}>
                    <div className="flex items-center justify-between mb-1">
                      <span className="text-sm font-medium text-gray-700">
                        Joint {index + 1}
                      </span>
                      <span className="text-sm font-mono text-gray-900">
                        {formatPosition(position)}°
                      </span>
                    </div>
                    <div className="w-full bg-gray-200 rounded-full h-2">
                      <div
                        className="bg-blue-600 h-2 rounded-full transition-all duration-300"
                        style={{ width: `${Math.max(0, Math.min(100, normalized))}%` }}
                      />
                    </div>
                  </div>
                )
              })}
            </div>
          </div>
        </Card>

        {/* Velocity Indicators */}
        <Card variant="elevated">
          <div className="space-y-4">
            <h3 className="text-lg font-semibold text-gray-900">관절 속도 모니터</h3>
            <div className="space-y-3">
              {[0, 1, 2, 3, 4, 5].map((index) => {
                const velocity = Math.abs(jointStates.velocity[index] || 0)
                const maxVelocity = 100 // 최대 속도 가정 (deg/s)
                const percentage = (velocity / maxVelocity) * 100

                return (
                  <div key={index}>
                    <div className="flex items-center justify-between mb-1">
                      <span className="text-sm font-medium text-gray-700">
                        Joint {index + 1}
                      </span>
                      <span className="text-sm font-mono text-gray-900">
                        {formatPosition(velocity)} deg/s
                      </span>
                    </div>
                    <div className="w-full bg-gray-200 rounded-full h-2">
                      <div
                        className={`h-2 rounded-full transition-all duration-300 ${
                          velocity > 50 ? 'bg-red-500' : velocity > 20 ? 'bg-yellow-500' : 'bg-green-500'
                        }`}
                        style={{ width: `${Math.min(100, percentage)}%` }}
                      />
                    </div>
                  </div>
                )
              })}
            </div>
          </div>
        </Card>
      </div>

      {/* Robot Control */}
      <Card variant="elevated">
        <div className="space-y-4">
          <div>
            <h3 className="text-xl font-bold text-gray-900 mb-1">로봇 제어</h3>
            <p className="text-sm text-gray-600">로봇 동작을 직접 제어합니다</p>
          </div>

          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            {/* Pause Button */}
            <Button
              variant="secondary"
              size="lg"
              onClick={handlePause}
              disabled={sendingCommand || robotState?.status === 'paused'}
              className="flex flex-col items-center py-6"
            >
              <span className="text-3xl mb-2">⏸️</span>
              <span className="text-sm font-semibold">일시정지</span>
            </Button>

            {/* Resume Button */}
            <Button
              variant="primary"
              size="lg"
              onClick={handleResume}
              disabled={sendingCommand || robotState?.status !== 'paused'}
              className="flex flex-col items-center py-6"
            >
              <span className="text-3xl mb-2">▶️</span>
              <span className="text-sm font-semibold">재개</span>
            </Button>

            {/* Stop Button */}
            <Button
              variant="secondary"
              size="lg"
              onClick={handleStop}
              disabled={sendingCommand || robotState?.status === 'idle'}
              className="flex flex-col items-center py-6"
            >
              <span className="text-3xl mb-2">⏹️</span>
              <span className="text-sm font-semibold">정지</span>
            </Button>

            {/* Emergency Stop Button */}
            <Button
              variant="danger"
              size="lg"
              onClick={handleEmergencyStop}
              disabled={sendingCommand}
              className="flex flex-col items-center py-6"
            >
              <span className="text-3xl mb-2">🚨</span>
              <span className="text-sm font-semibold">긴급정지</span>
            </Button>
          </div>

          {/* Command Status */}
          {robotState?.desired_state && (
            <div className="bg-blue-50 border border-blue-200 rounded-lg p-3">
              <div className="flex items-center space-x-2">
                <span className="text-blue-600 font-semibold">전송된 명령:</span>
                <span className="text-blue-900">{robotState.desired_state}</span>
                {robotState.command_timestamp && (
                  <span className="text-blue-600 text-sm">
                    ({new Date(robotState.command_timestamp).toLocaleTimeString('ko-KR')})
                  </span>
                )}
              </div>
            </div>
          )}
        </div>
      </Card>

      {/* Quick Actions */}
      <Card variant="outlined" className="bg-gray-50">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-lg font-semibold text-gray-900 mb-1">빠른 작업</h3>
            <p className="text-sm text-gray-600">로봇 제어 및 모니터링</p>
          </div>
          <div className="flex space-x-3">
            <Link href="/tasks/monitor">
              <Button variant="secondary">작업 모니터</Button>
            </Link>
            <Link href="/job/material">
              <Button variant="primary">새 작업 시작</Button>
            </Link>
          </div>
        </div>
      </Card>
    </div>
  )
}
