/**
 * MCA-02-02: 손질 모드 선택 페이지
 */
import { useState, useEffect } from 'react'
import { useNavigate } from 'react-router-dom'
import { Card } from '@components/Card'
import { Button } from '@components/Button'
import { ProgressIndicator } from '@components/ProgressIndicator'
import { useJobConfigStore } from '@store/jobConfigStore'
import { modesApi } from '@services/api'
import './ModeSelection.css'

interface Mode {
  id: string
  name: string
  icon: string
  description: string
  estimatedTime: number
  supportedMaterials: string[]
  parameters: any
}

export default function ModeSelection() {
  const navigate = useNavigate()
  const { material, setMode } = useJobConfigStore()
  const [modes, setModes] = useState<Mode[]>([])
  const [selectedMode, setSelectedMode] = useState<Mode | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    if (!material) {
      navigate('/job/new/material')
      return
    }
    loadModes()
  }, [material, navigate])

  const loadModes = async () => {
    try {
      const response: any = await modesApi.getAll(material?.id)
      if (response.success) {
        setModes(response.data)
      }
    } catch (error) {
      console.error('Failed to load modes:', error)
    } finally {
      setLoading(false)
    }
  }

  const handleModeClick = (mode: Mode) => {
    setSelectedMode(mode)
  }

  const handleNext = () => {
    if (selectedMode) {
      setMode(selectedMode)
      navigate('/job/new/parameters')
    }
  }

  const handleBack = () => {
    navigate('/job/new/material')
  }

  if (loading) {
    return (
      <div className="container">
        <div className="loading">모드 목록을 불러오는 중...</div>
      </div>
    )
  }

  return (
    <div className="mode-selection">
      <div className="page-header">
        <Button variant="ghost" onClick={handleBack}>
          ← 뒤로
        </Button>
        <h1>손질 모드 선택</h1>
        <ProgressIndicator current={2} total={4} />
      </div>

      <div className="page-content">
        <div className="selected-material-badge">
          선택한 재료: {material?.emoji} {material?.name}
        </div>

        <h2 className="section-title">어떻게 손질하시겠어요?</h2>

        <div className="mode-list">
          {modes.map((mode) => (
            <Card
              key={mode.id}
              selectable
              selected={selectedMode?.id === mode.id}
              onClick={() => handleModeClick(mode)}
            >
              <div className="mode-card-content">
                <div className="mode-icon">{mode.icon}</div>
                <div className="mode-info">
                  <h3 className="mode-name">{mode.name}</h3>
                  <p className="mode-description">{mode.description}</p>
                  <div className="mode-meta">
                    <span className="mode-time">
                      예상 시간: 약 {Math.floor(mode.estimatedTime / 60)}분
                    </span>
                  </div>
                </div>
              </div>
            </Card>
          ))}
        </div>
      </div>

      <div className="page-footer">
        <Button
          variant="primary"
          size="lg"
          disabled={!selectedMode}
          onClick={handleNext}
          fullWidth
        >
          다음 →
        </Button>
      </div>
    </div>
  )
}
