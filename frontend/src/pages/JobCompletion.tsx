/**
 * MCA-04: 작업 완료 페이지
 */
import { useState, useEffect } from 'react'
import { useNavigate, useParams } from 'react-router-dom'
import { Button } from '@components/Button'
import { Card } from '@components/Card'
import { jobsApi } from '@services/api'
import './JobCompletion.css'

export default function JobCompletion() {
  const navigate = useNavigate()
  const { jobId } = useParams<{ jobId: string }>()
  const [job, setJob] = useState<any>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    loadJobDetails()
  }, [jobId])

  const loadJobDetails = async () => {
    try {
      const response: any = await jobsApi.getById(jobId!)
      if (response.success) {
        setJob(response.data)
      }
    } catch (error) {
      console.error('Failed to load job:', error)
    } finally {
      setLoading(false)
    }
  }

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60)
    const secs = Math.floor(seconds % 60)
    return mins > 0 ? `${mins}분 ${secs}초` : `${secs}초`
  }

  const handleRepeat = () => {
    // 같은 설정으로 새 작업 시작
    navigate('/job/new/confirm')
  }

  const handleNewJob = () => {
    navigate('/job/new/material')
  }

  const handleGoHome = () => {
    navigate('/')
  }

  if (loading) {
    return (
      <div className="container">
        <div className="loading">작업 정보를 불러오는 중...</div>
      </div>
    )
  }

  return (
    <div className="job-completion">
      <div className="completion-animation">
        <div className="checkmark-circle">
          <div className="checkmark">✓</div>
        </div>
        <h1 className="completion-title">작업 완료!</h1>
      </div>

      <div className="completion-content">
        <Card variant="elevated">
          <h2 className="result-title">작업 결과</h2>

          <div className="result-grid">
            <div className="result-item">
              <span className="result-label">재료</span>
              <span className="result-value">{job?.material}</span>
            </div>

            <div className="result-item">
              <span className="result-label">모드</span>
              <span className="result-value">{job?.mode}</span>
            </div>

            <div className="result-divider" />

            <div className="result-item">
              <span className="result-label">작업 시간</span>
              <span className="result-value highlight">
                {formatTime(job?.actualTime || 0)}
              </span>
            </div>

            <div className="result-item">
              <span className="result-label">상태</span>
              <span className="result-value success">✓ 정상 완료</span>
            </div>
          </div>

          {job?.estimatedTime && job?.actualTime < job.estimatedTime && (
            <div className="insight">
              💡 예상 시간({formatTime(job.estimatedTime)})보다{' '}
              {formatTime(job.estimatedTime - job.actualTime)} 빨랐습니다
            </div>
          )}
        </Card>

        <div className="action-buttons">
          <Button
            variant="secondary"
            size="lg"
            onClick={handleRepeat}
            fullWidth
          >
            🔄 같은 작업 반복하기
          </Button>
          <Button
            variant="primary"
            size="lg"
            onClick={handleNewJob}
            fullWidth
          >
            ➕ 새 작업 시작하기
          </Button>
        </div>

        <Button
          variant="ghost"
          onClick={handleGoHome}
          fullWidth
        >
          🏠 메인으로
        </Button>
      </div>
    </div>
  )
}
