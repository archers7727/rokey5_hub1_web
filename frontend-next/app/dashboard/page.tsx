'use client'

import { useState, useEffect } from 'react'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { useRobotStateRealtime } from '@/hooks/useRobotStateRealtime'

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
        // Transform jobs data for dashboard
        const jobs = result.data || []
        setDashboardData({
          todayStats: {
            jobCount: jobs.length,
            totalTime: jobs.reduce((acc: number, job: any) => acc + (job.actual_time || 0), 0)
          },
          recentJobs: jobs.slice(0, 5).map((job: any) => ({
            id: job.id,
            material: job.material_id,
            mode: job.mode_id,
            completedAt: job.completed_at,
            duration: job.actual_time || 0
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
    const date = new Date(dateString)
    const now = new Date()
    const diff = Math.floor((now.getTime() - date.getTime()) / 1000)

    if (diff < 60) return `${diff}초 전`
    if (diff < 3600) return `${Math.floor(diff / 60)}분 전`
    if (diff < 86400) return `${Math.floor(diff / 3600)}시간 전`
    return `${Math.floor(diff / 86400)}일 전`
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

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {/* 로봇 상태 카드 */}
        <Card variant="elevated">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">로봇 상태</h3>
          <div className="flex items-center space-x-4">
            <div className={`w-4 h-4 rounded-full ${getStatusColor()}`} />
            <div>
              <div className="text-xl font-semibold text-gray-900">{getStatusText()}</div>
              <div className="text-sm text-gray-500">Doosan M0609</div>
            </div>
          </div>
        </Card>

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
