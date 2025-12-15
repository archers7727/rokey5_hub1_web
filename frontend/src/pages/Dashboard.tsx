/**
 * MCA-01: 메인 대시보드
 */
import { useState, useEffect } from 'react'
import { useNavigate } from 'react-router-dom'
import { Card } from '@components/Card'
import { Button } from '@components/Button'
import { dashboardApi } from '@services/api'
import { useRobotStore } from '@store/robotStore'
import { useWebSocket } from '@hooks/useWebSocket'
import './Dashboard.css'

export default function Dashboard() {
  const navigate = useNavigate()
  const { status: robotStatus } = useRobotStore()
  const [dashboardData, setDashboardData] = useState<any>(null)
  const [loading, setLoading] = useState(true)

  // WebSocket으로 로봇 상태 실시간 수신
  useWebSocket('/api/robot/ws/status', {
    onMessage: (data) => {
      useRobotStore.getState().updateRobotState(data)
    },
  })

  useEffect(() => {
    loadDashboardData()
  }, [])

  const loadDashboardData = async () => {
    try {
      const response: any = await dashboardApi.getData()
      if (response.success) {
        setDashboardData(response.data)
      }
    } catch (error) {
      console.error('Failed to load dashboard data:', error)
    } finally {
      setLoading(false)
    }
  }

  const getStatusColor = () => {
    const colors = {
      idle: 'var(--gray-500)',
      working: 'var(--status-normal)',
      emergency_stop: 'var(--status-danger)',
      user_stop: 'var(--status-warning)',
      error: 'var(--status-danger)',
    }
    return colors[robotStatus as keyof typeof colors] || colors.idle
  }

  const getStatusText = () => {
    const texts = {
      idle: '대기 중',
      working: '작동 중',
      emergency_stop: '비상 정지',
      user_stop: '일시 정지',
      error: '에러',
    }
    return texts[robotStatus as keyof typeof texts] || '알 수 없음'
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
      <div className="container">
        <div className="loading">대시보드 로딩 중...</div>
      </div>
    )
  }

  return (
    <div className="dashboard">
      <div className="dashboard-header">
        <h1>두산 로봇팔 재료 손질 시스템</h1>
      </div>

      <div className="dashboard-grid">
        {/* 로봇 상태 카드 */}
        <Card variant="elevated">
          <h3 className="card-title">로봇 상태</h3>
          <div className="robot-status">
            <div
              className="status-indicator"
              style={{ backgroundColor: getStatusColor() }}
            />
            <div className="status-info">
              <div className="status-text">{getStatusText()}</div>
              <div className="status-model">Doosan M1013</div>
            </div>
          </div>
        </Card>

        {/* 오늘의 통계 카드 */}
        <Card variant="elevated">
          <h3 className="card-title">오늘의 통계</h3>
          <div className="stats-grid">
            <div className="stat-item">
              <div className="stat-value">{dashboardData?.todayStats?.jobCount || 0}</div>
              <div className="stat-label">작업 수</div>
            </div>
            <div className="stat-divider" />
            <div className="stat-item">
              <div className="stat-value">
                {formatTime(dashboardData?.todayStats?.totalTime || 0)}
              </div>
              <div className="stat-label">총 시간</div>
            </div>
          </div>
        </Card>

        {/* 작업 시작 버튼 */}
        <div className="start-job-section">
          <Button
            variant="primary"
            size="lg"
            onClick={() => navigate('/job/new/material')}
            fullWidth
          >
            🚀 작업 시작하기
          </Button>
        </div>

        {/* 최근 작업 이력 */}
        <div className="recent-jobs-section">
          <h2 className="section-title">최근 작업 이력</h2>
          <div className="recent-jobs-grid">
            {dashboardData?.recentJobs?.map((job: any) => (
              <Card key={job.id} variant="outlined">
                <div className="job-card-content">
                  <div className="job-info">
                    <h4 className="job-title">
                      {job.material} - {job.mode}
                    </h4>
                    <div className="job-meta">
                      <span>{formatTimeAgo(job.completedAt)}</span>
                      <span>•</span>
                      <span>{formatTime(job.duration)}</span>
                    </div>
                  </div>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => navigate('/job/new/confirm')}
                  >
                    재실행
                  </Button>
                </div>
              </Card>
            ))}

            {(!dashboardData?.recentJobs || dashboardData.recentJobs.length === 0) && (
              <div className="empty-state">
                <p>최근 작업 이력이 없습니다</p>
                <Button
                  variant="secondary"
                  onClick={() => navigate('/job/new/material')}
                >
                  첫 작업 시작하기
                </Button>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  )
}
