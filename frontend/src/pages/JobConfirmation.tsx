/**
 * MCA-02-04: 작업 확인 페이지 (3D 제외)
 */
import { useState, useEffect } from 'react'
import { useNavigate } from 'react-router-dom'
import { Button } from '@components/Button'
import { Card } from '@components/Card'
import { ProgressIndicator } from '@components/ProgressIndicator'
import { useJobConfigStore } from '@store/jobConfigStore'
import { jobsApi } from '@services/api'
import './JobConfirmation.css'

export default function JobConfirmation() {
  const navigate = useNavigate()
  const { material, mode, parameters, estimatedTime, reset } = useJobConfigStore()
  const [creating, setCreating] = useState(false)

  useEffect(() => {
    if (!material || !mode || !parameters) {
      navigate('/job/new/material')
      return
    }
  }, [material, mode, parameters, navigate])

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60)
    const secs = Math.floor(seconds % 60)
    return mins > 0 ? `${mins}분 ${secs}초` : `${secs}초`
  }

  const getSizeLabel = () => {
    const labels = { small: '작음', medium: '보통', large: '큼' }
    return labels[parameters?.materialSize || 'medium']
  }

  const getSizeCm = () => {
    if (!material || !parameters) return 0
    return material.sizes[parameters.materialSize]
  }

  const handleExecute = async () => {
    setCreating(true)
    try {
      const jobData = {
        material: material?.id,
        mode: mode?.id,
        parameters,
        estimatedTime,
      }

      const response: any = await jobsApi.create(jobData)

      if (response.success) {
        const jobId = response.data.id
        // 작업 생성 성공, 즉시 시작
        await jobsApi.start(jobId)
        // 작업 설정 초기화
        reset()
        // 모니터링 페이지로 이동
        navigate(`/job/monitor/${jobId}`)
      }
    } catch (error) {
      console.error('Failed to create job:', error)
      alert('작업 생성에 실패했습니다.')
    } finally {
      setCreating(false)
    }
  }

  const handleBack = () => {
    navigate('/job/new/parameters')
  }

  return (
    <div className="job-confirmation">
      <div className="page-header">
        <Button variant="ghost" onClick={handleBack} disabled={creating}>
          ← 뒤로
        </Button>
        <h1>작업 확인</h1>
        <ProgressIndicator current={4} total={4} />
      </div>

      <div className="page-content">
        <div className="confirmation-layout">
          {/* 작업 요약 */}
          <Card variant="elevated">
            <h2 className="summary-title">작업 요약</h2>

            <div className="summary-grid">
              <div className="summary-item">
                <span className="summary-label">재료</span>
                <span className="summary-value">
                  {material?.emoji} {material?.name}
                </span>
              </div>

              <div className="summary-item">
                <span className="summary-label">모드</span>
                <span className="summary-value">
                  {mode?.icon} {mode?.name}
                </span>
              </div>

              <div className="summary-item">
                <span className="summary-label">크기</span>
                <span className="summary-value">
                  {getSizeLabel()} ({getSizeCm()}cm)
                </span>
              </div>

              {parameters?.fryingParams && (
                <div className="summary-item">
                  <span className="summary-label">조각</span>
                  <span className="summary-value">{parameters.fryingParams.pieces}조각</span>
                </div>
              )}

              {parameters?.slicingParams && (
                <div className="summary-item">
                  <span className="summary-label">두께</span>
                  <span className="summary-value">{parameters.slicingParams.thickness}mm</span>
                </div>
              )}

              <div className="summary-item">
                <span className="summary-label">반복</span>
                <span className="summary-value">{parameters?.repeatCount}회</span>
              </div>

              <div className="summary-divider"></div>

              <div className="summary-item highlight">
                <span className="summary-label">예상 시간</span>
                <span className="summary-value">{formatTime(estimatedTime)}</span>
              </div>
            </div>
          </Card>

          {/* 3D 미리보기 영역 (제외) */}
          <div className="preview-placeholder">
            <div className="placeholder-content">
              <div className="placeholder-icon">📊</div>
              <h3>3D 미리보기</h3>
              <p>3D 시각화는 향후 업데이트 예정입니다</p>
            </div>
          </div>
        </div>
      </div>

      <div className="page-footer">
        <Button
          variant="primary"
          size="lg"
          onClick={handleExecute}
          disabled={creating}
          fullWidth
        >
          {creating ? '작업 생성 중...' : '🔵 즉시 실행'}
        </Button>
      </div>
    </div>
  )
}
