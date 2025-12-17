'use client'

import { useState, useEffect } from 'react'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { useRobotStateRealtime } from '@/hooks/useRobotStateRealtime'

export const dynamic = 'force-dynamic'

interface DashboardData {
  todayStats?: {
    jobCount: number
    totalTime: number
  }
  recentJobs?: Array<{
    id: string
    material: string
    mode: string
    completedAt: string
    duration: number
  }>
}

export default function Dashboard() {
  const robotState = useRobotStateRealtime()
  const [dashboardData, setDashboardData] = useState<DashboardData | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    loadDashboardData()
  }, [])

  const loadDashboardData = async () => {
    try {
      const response = await fetch('/api/jobs')
      const result = await response.json()

      if (result.success) {
        // Transform tasks data for dashboard
        const tasks = result.data || []

        // 완료된 작업만 필터링
        const completedTasks = tasks.filter((task: any) => task.status === 'completed')

        // 오늘 날짜 필터링
        const today = new Date()
        today.setHours(0, 0, 0, 0)
        const tomorrow = new Date(today)
        tomorrow.setDate(tomorrow.getDate() + 1)

        const todayTasks = completedTasks.filter((task: any) => {
          // completed_at이 있으면 사용, 없으면 created_at 사용
          const dateToCheck = task.completed_at || task.created_at
          if (!dateToCheck) return false

          const taskDate = new Date(dateToCheck)
          // 오늘 00:00:00 ~ 내일 00:00:00 사이
          return taskDate >= today && taskDate < tomorrow
        })

        setDashboardData({
          todayStats: {
            jobCount: todayTasks.length,
            totalTime: todayTasks.reduce((acc: number, task: any) => acc + (task.estimated_time || 0), 0)
          },
          recentJobs: completedTasks.slice(0, 5).map((task: any) => ({
            id: task.id,
            material: task.material_id,
            mode: task.mode_id,
            completedAt: task.completed_at || task.created_at,
            duration: task.estimated_time || 0
          }))
        })
      }
    } catch (error) {
      console.error('Failed to load dashboard data:', error)
    } finally {
      setLoading(false)
    }
  }

  const getStatusColor = () => {
    if (!robotState) return 'bg-gray-500'

    const colors: Record<string, string> = {
      idle: 'bg-gray-500',
      running: 'bg-green-500',
      paused: 'bg-yellow-500',
      error: 'bg-red-500',
    }
    return colors[robotState.status] || 'bg-gray-500'
  }

  const getStatusText = () => {
    if (!robotState) return '연결 중...'

    const texts: Record<string, string> = {
      idle: '대기 중',
      running: '작동 중',
      paused: '일시 정지',
      error: '에러',
    }
    return texts[robotState.status] || '알 수 없음'
  }

  const formatTime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600)
    const mins = Math.floor((seconds % 3600) / 60)
    return hours > 0 ? `${hours}시간 ${mins}분` : `${mins}분`
  }

  const formatTimeAgo = (dateString: string) => {
    if (!dateString) return '방금 전'

    const date = new Date(dateString)

    // 유효하지 않은 날짜 체크
    if (isNaN(date.getTime())) return '방금 전'

    const now = new Date()
    const diff = Math.floor((now.getTime() - date.getTime()) / 1000)

    // 음수면 (미래 날짜) 방금 전으로 표시
    if (diff < 0) return '방금 전'

    if (diff < 60) return `${diff}초 전`
    if (diff < 3600) return `${Math.floor(diff / 60)}분 전`
    if (diff < 86400) return `${Math.floor(diff / 3600)}시간 전`

    // 24시간 이상일 때만 일수로 표시
    const days = Math.floor(diff / 86400)
    return `${days}일 전`
  }

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-lg text-gray-600">대시보드 로딩 중...</div>
      </div>
    )
  }

  return (
    <div className="space-y-8">
      <div className="flex items-center justify-between">
        <h1 className="text-3xl font-bold text-gray-900">Mr.Chef Assistant</h1>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        {/* 로봇 상태 카드 */}
        <Link href="/robot/status" className="block">
          <Card variant="elevated" className="cursor-pointer hover:shadow-lg transition-shadow">
            <h3 className="text-lg font-semibold text-gray-900 mb-4">로봇 상태</h3>
            <div className="flex items-center space-x-4">
              <div className={`w-4 h-4 rounded-full ${getStatusColor()}`} />
              <div>
                <div className="text-xl font-semibold text-gray-900">{getStatusText()}</div>
                <div className="text-sm text-gray-500">Doosan M0609</div>
              </div>
            </div>
          </Card>
        </Link>

        {/* 작업 모니터 카드 */}
        <Link href="/tasks/monitor" className="block">
          <Card variant="elevated" className="cursor-pointer hover:shadow-lg transition-shadow">
            <h3 className="text-lg font-semibold text-gray-900 mb-4">작업 모니터</h3>
            <div className="flex items-center space-x-4">
              <div className="text-3xl">📊</div>
              <div>
                <div className="text-xl font-semibold text-gray-900">작업 현황</div>
                <div className="text-sm text-gray-500">실시간 모니터링</div>
              </div>
            </div>
          </Card>
        </Link>

        {/* 오늘의 통계 카드 */}
        <Card variant="elevated">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">오늘의 통계</h3>
          <div className="flex items-center justify-around">
            <div className="text-center">
              <div className="text-3xl font-bold text-blue-600">
                {dashboardData?.todayStats?.jobCount || 0}
              </div>
              <div className="text-sm text-gray-500">작업 수</div>
            </div>
            <div className="w-px h-12 bg-gray-200" />
            <div className="text-center">
              <div className="text-3xl font-bold text-blue-600">
                {formatTime(dashboardData?.todayStats?.totalTime || 0)}
              </div>
              <div className="text-sm text-gray-500">총 시간</div>
            </div>
          </div>
        </Card>

        {/* 작업 시작 버튼 */}
        <Card variant="elevated" className="flex items-center justify-center">
          <Link href="/job/material" className="w-full">
            <Button variant="primary" size="lg" fullWidth>
              🚀 작업 시작하기
            </Button>
          </Link>
        </Card>
      </div>

      {/* 최근 작업 이력 */}
      <div>
        <h2 className="text-2xl font-bold text-gray-900 mb-4">최근 작업 이력</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {dashboardData?.recentJobs?.map((job) => (
            <Card key={job.id} variant="outlined">
              <div className="flex items-center justify-between">
                <div>
                  <h4 className="font-semibold text-gray-900">
                    {job.material} - {job.mode}
                  </h4>
                  <div className="text-sm text-gray-500 mt-1">
                    <span>{formatTimeAgo(job.completedAt)}</span>
                    <span className="mx-1">•</span>
                    <span>{formatTime(job.duration)}</span>
                  </div>
                </div>
                <Button variant="ghost" size="sm">
                  재실행
                </Button>
              </div>
            </Card>
          ))}

          {(!dashboardData?.recentJobs || dashboardData.recentJobs.length === 0) && (
            <Card variant="outlined" className="col-span-full">
              <div className="text-center py-8">
                <p className="text-gray-500 mb-4">최근 작업 이력이 없습니다</p>
                <Link href="/job/material">
                  <Button variant="secondary">
                    첫 작업 시작하기
                  </Button>
                </Link>
              </div>
            </Card>
          )}
        </div>
      </div>
    </div>
  )
}
