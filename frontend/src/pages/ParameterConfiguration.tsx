/**
 * MCA-02-03: 작업 파라미터 설정 페이지
 */
import { useState, useEffect } from 'react'
import { useNavigate } from 'react-router-dom'
import { Button } from '@components/Button'
import { ProgressIndicator } from '@components/ProgressIndicator'
import { RadioGroup } from '@components/RadioGroup'
import { Select } from '@components/Select'
import { NumberInput } from '@components/NumberInput'
import { Card } from '@components/Card'
import { useJobConfigStore } from '@store/jobConfigStore'
import './ParameterConfiguration.css'

type MaterialSize = 'small' | 'medium' | 'large'

export default function ParameterConfiguration() {
  const navigate = useNavigate()
  const { material, mode, setParameters, setEstimatedTime } = useJobConfigStore()

  const [materialSize, setMaterialSize] = useState<MaterialSize>('medium')
  const [pieces, setPieces] = useState(4) // 튀김 모드
  const [thickness, setThickness] = useState(10) // 썰기 모드 (mm)
  const [repeatCount, setRepeatCount] = useState(1)

  // 예상 정보 계산
  const [estimatedCuts, setEstimatedCuts] = useState(0)
  const [singleJobTime, setSingleJobTime] = useState(0)
  const [totalJobTime, setTotalJobTime] = useState(0)

  useEffect(() => {
    if (!material || !mode) {
      navigate('/job/new/material')
      return
    }
  }, [material, mode, navigate])

  useEffect(() => {
    calculateEstimation()
  }, [materialSize, pieces, thickness, repeatCount, mode])

  const getSizeInCm = () => {
    if (!material) return 8
    return material.sizes[materialSize]
  }

  const calculateEstimation = () => {
    if (!mode) return

    let cuts = 0
    let baseTime = mode.estimatedTime // seconds

    if (mode.id === 'frying') {
      // 튀김 모드: 조각 수에 따른 절단 횟수
      const cutMap: { [key: number]: number } = { 2: 1, 4: 2, 6: 3, 8: 3 }
      cuts = cutMap[pieces] || 2
    } else if (mode.id === 'slicing') {
      // 썰기 모드: 재료 크기 / 두께
      const materialMm = getSizeInCm() * 10
      cuts = Math.floor(materialMm / thickness)
    }

    // 절단 횟수에 따른 시간 추가
    const timePerCut = 0.5 // 절단당 0.5초
    const singleTime = baseTime + cuts * timePerCut
    const totalTime = singleTime * repeatCount

    setEstimatedCuts(cuts)
    setSingleJobTime(singleTime)
    setTotalJobTime(totalTime)
  }

  const handleNext = () => {
    const parameters = {
      materialSize,
      repeatCount,
      ...(mode?.id === 'frying'
        ? { fryingParams: { pieces } }
        : { slicingParams: { thickness } }),
    }

    setParameters(parameters)
    setEstimatedTime(totalJobTime)
    navigate('/job/new/confirm')
  }

  const handleBack = () => {
    navigate('/job/new/mode')
  }

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60)
    const secs = Math.floor(seconds % 60)
    return mins > 0 ? `${mins}분 ${secs}초` : `${secs}초`
  }

  return (
    <div className="parameter-configuration">
      <div className="page-header">
        <Button variant="ghost" onClick={handleBack}>
          ← 뒤로
        </Button>
        <h1>작업 파라미터 설정</h1>
        <ProgressIndicator current={3} total={4} />
      </div>

      <div className="page-content">
        <div className="breadcrumb">
          {material?.emoji} {material?.name} &gt; {mode?.icon} {mode?.name}
        </div>

        {/* 재료 크기 */}
        <div className="param-section">
          <h3 className="param-title">재료 크기</h3>
          <RadioGroup
            name="materialSize"
            value={materialSize}
            onChange={(value) => setMaterialSize(value as MaterialSize)}
            options={[
              {
                value: 'small',
                label: '작음',
                sublabel: `(${material?.sizes.small || 6}cm)`,
              },
              {
                value: 'medium',
                label: '보통',
                sublabel: `(${material?.sizes.medium || 8}cm)`,
              },
              {
                value: 'large',
                label: '큼',
                sublabel: `(${material?.sizes.large || 10}cm)`,
              },
            ]}
          />
        </div>

        {/* 모드별 파라미터 */}
        {mode?.id === 'frying' && (
          <div className="param-section">
            <h3 className="param-title">조각 수</h3>
            <Select
              value={pieces}
              onChange={setPieces}
              options={[2, 4, 6, 8]}
              label={`${pieces}조각으로 나누기`}
            />
          </div>
        )}

        {mode?.id === 'slicing' && (
          <div className="param-section">
            <h3 className="param-title">두께 선택</h3>
            <div className="slider-wrapper">
              <input
                type="range"
                min="5"
                max="30"
                step="5"
                value={thickness}
                onChange={(e) => setThickness(Number(e.target.value))}
                className="slider"
              />
              <div className="slider-labels">
                <span>5mm</span>
                <span>10mm</span>
                <span>15mm</span>
                <span>20mm</span>
                <span>25mm</span>
                <span>30mm</span>
              </div>
              <div className="slider-value">현재: {thickness}mm</div>
            </div>
          </div>
        )}

        {/* 반복 횟수 */}
        <div className="param-section">
          <h3 className="param-title">반복 횟수</h3>
          <NumberInput
            value={repeatCount}
            onChange={setRepeatCount}
            min={1}
            max={10}
            label={`같은 작업을 ${repeatCount}번 반복합니다`}
          />
        </div>

        {/* 예상 정보 */}
        <Card variant="elevated">
          <h3 className="estimation-title">예상 정보</h3>
          <div className="estimation-grid">
            <div className="estimation-item">
              <span className="estimation-label">예상 작업 횟수:</span>
              <span className="estimation-value">{estimatedCuts}번 절단</span>
            </div>
            <div className="estimation-item">
              <span className="estimation-label">단일 작업 시간:</span>
              <span className="estimation-value">약 {formatTime(singleJobTime)}</span>
            </div>
            <div className="estimation-item">
              <span className="estimation-label">총 작업 시간:</span>
              <span className="estimation-value highlight">
                약 {formatTime(totalJobTime)}
              </span>
            </div>
          </div>
        </Card>
      </div>

      <div className="page-footer">
        <Button variant="primary" size="lg" onClick={handleNext} fullWidth>
          다음 →
        </Button>
      </div>
    </div>
  )
}
