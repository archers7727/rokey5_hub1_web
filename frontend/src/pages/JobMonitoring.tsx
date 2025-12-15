/**
 * MCA-03: 작업 실행 모니터링 페이지 (3D 제외)
 */
import { useState, useEffect } from 'react'
import { useNavigate, useParams } from 'react-router-dom'
import { Button } from '@components/Button'
import { Card } from '@components/Card'
import { ProgressBar } from '@components/ProgressBar'
import { useWebSocket } from '@hooks/useWebSocket'
import { jobsApi } from '@services/api'
import './JobMonitoring.css'

export default function JobMonitoring() {
  const navigate = useNavigate()
  const { jobId } = useParams<{ jobId: string }>()

  const [isPaused, setIsPaused] = useState(false)
  const [progress, setProgress] = useState(0)
  const [currentStep, setCurrentStep] = useState(0)
  const [totalSteps, setTotalSteps] = useState(0)
  const [stepDescription, setStepDescription] = useState('')
  const [jointAngles, setJointAngles] = useState<number[]>([0, 0, 90, 0, 90, 0])
  const [tcpPosition, setTcpPosition] = useState({ x: 0.45, y: 0, z: 0.68 })
  const [speed, setSpeed] = useState(0)
  const [elapsedTime, setElapsedTime] = useState(0)

  // WebSocket 연결
  useWebSocket(`/api/jobs/ws/${jobId}`, {
    onMessage: (data) => {
      setProgress(data.progress || 0)
      setCurrentStep(data.currentStep || 0)
      setTotalSteps(data.totalSteps || 10)
      setStepDescription(data.stepDescription || '')
      setJointAngles(data.jointAngles || [0, 0, 90, 0, 90, 0])
      setTcpPosition(data.tcpPosition || { x: 0.45, y: 0, z: 0.68 })
      setSpeed(data.speed || 0)

      // 작업 완료 시 완료 페이지로 이동
      if (data.status === 'completed') {
        setTimeout(() => {
          navigate(`/job/complete/${jobId}`)
        }, 1000)
      }
    },
  })

  // 경과 시간 계산
  useEffect(() => {
    const interval = setInterval(() => {
      setElapsedTime((prev) => prev + 1)
    }, 1000)
    return () => clearInterval(interval)
  }, [])

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60)
    const secs = seconds % 60
    return `${mins}:${secs.toString().padStart(2, '0')}`
  }

  const handlePauseResume = async () => {
    try {
      if (isPaused) {
        await jobsApi.resume(jobId!)
      } else {
        await jobsApi.pause(jobId!)
      }
      setIsPaused(!isPaused)
    } catch (error) {
      console.error('Failed to pause/resume:', error)
    }
  }

  const handleStop = async () => {
    if (confirm('작업을 정지하시겠습니까?')) {
      try {
        await jobsApi.stop(jobId!)
        navigate('/')
      } catch (error) {
        console.error('Failed to stop:', error)
      }
    }
  }

  const handleEmergencyStop = async () => {
    if (confirm('비상 정지하시겠습니까?')) {
      try {
        await jobsApi.stop(jobId!)
        alert('비상 정지되었습니다.')
        navigate('/robot/status')
      } catch (error) {
        console.error('Failed to emergency stop:', error)
      }
    }
  }

  return (
    <div className="job-monitoring">
      <div className="monitoring-header">
        <h1>작업 진행 중...</h1>
        <div className="control-buttons">
          <Button
            variant={isPaused ? 'primary' : 'secondary'}
            onClick={handlePauseResume}
          >
            {isPaused ? '▶ 재개' : '⏸ 일시정지'}
          </Button>
          <Button variant="secondary" onClick={handleStop}>
            ⏹ 정지
          </Button>
          <Button variant="danger" onClick={handleEmergencyStop}>
            🛑 비상정지
          </Button>
        </div>
      </div>

      <div className="monitoring-content">
        {/* 진행률 표시 */}
        <Card variant="elevated">
          <ProgressBar progress={progress} />
          <div className="progress-info">
            <span>
              현재: {stepDescription} ({currentStep}/{totalSteps})
            </span>
            <span>경과 시간: {formatTime(elapsedTime)}</span>
          </div>
        </Card>

        <div className="monitoring-grid">
          {/* 로봇 상태 (3D 대신 데이터 표시) */}
          <Card>
            <h3 className="card-title">로봇 상태</h3>
            <div className="status-grid">
              <div className="status-item">
                <span className="status-label">현재 단계</span>
                <span className="status-value">{stepDescription}</span>
              </div>

              <div className="status-divider" />

              <div className="status-section">
                <h4>관절 각도</h4>
                {jointAngles.map((angle, i) => (
                  <div key={i} className="joint-item">
                    <span>Joint {i + 1}:</span>
                    <span className="joint-value">{angle.toFixed(1)}°</span>
                  </div>
                ))}
              </div>

              <div className="status-divider" />

              <div className="status-section">
                <h4>TCP 위치</h4>
                <div className="joint-item">
                  <span>X:</span>
                  <span className="joint-value">{tcpPosition.x.toFixed(3)} m</span>
                </div>
                <div className="joint-item">
                  <span>Y:</span>
                  <span className="joint-value">{tcpPosition.y.toFixed(3)} m</span>
                </div>
                <div className="joint-item">
                  <span>Z:</span>
                  <span className="joint-value">{tcpPosition.z.toFixed(3)} m</span>
                </div>
              </div>

              <div className="status-divider" />

              <div className="status-item">
                <span className="status-label">속도</span>
                <span className="status-value">{speed.toFixed(2)} m/s</span>
              </div>
            </div>
          </Card>

          {/* 3D 뷰 플레이스홀더 */}
          <div className="preview-placeholder">
            <div className="placeholder-content">
              <div className="placeholder-icon">🤖</div>
              <h3>실시간 로봇 뷰</h3>
              <p>3D 시각화는 향후 업데이트 예정입니다</p>
              <div className="placeholder-status">
                진행률: {Math.round(progress)}%
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
