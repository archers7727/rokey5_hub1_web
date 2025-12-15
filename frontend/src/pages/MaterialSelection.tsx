/**
 * MCA-02-01: 재료 선택 페이지
 */
import { useState, useEffect } from 'react'
import { useNavigate } from 'react-router-dom'
import { Card } from '@components/Card'
import { Button } from '@components/Button'
import { ProgressIndicator } from '@components/ProgressIndicator'
import { useJobConfigStore } from '@store/jobConfigStore'
import { materialsApi } from '@services/api'
import './MaterialSelection.css'

interface Material {
  id: string
  name: string
  emoji: string
  description: string
  category: string
  sizes: {
    small: number
    medium: number
    large: number
  }
}

export default function MaterialSelection() {
  const navigate = useNavigate()
  const { setMaterial } = useJobConfigStore()
  const [materials, setMaterials] = useState<Material[]>([])
  const [selectedMaterial, setSelectedMaterial] = useState<Material | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    loadMaterials()
  }, [])

  const loadMaterials = async () => {
    try {
      const response: any = await materialsApi.getAll()
      if (response.success) {
        setMaterials(response.data)
      }
    } catch (error) {
      console.error('Failed to load materials:', error)
    } finally {
      setLoading(false)
    }
  }

  const handleMaterialClick = (material: Material) => {
    setSelectedMaterial(material)
  }

  const handleNext = () => {
    if (selectedMaterial) {
      setMaterial(selectedMaterial)
      navigate('/job/new/mode')
    }
  }

  if (loading) {
    return (
      <div className="container">
        <div className="loading">재료 목록을 불러오는 중...</div>
      </div>
    )
  }

  return (
    <div className="material-selection">
      <div className="page-header">
        <Button variant="ghost" onClick={() => navigate('/')}>
          ← 뒤로
        </Button>
        <h1>재료 선택</h1>
        <ProgressIndicator current={1} total={4} />
      </div>

      <div className="page-content">
        <h2 className="section-title">어떤 재료를 손질하시겠어요?</h2>

        <div className="material-grid">
          {materials.map((material) => (
            <Card
              key={material.id}
              selectable
              selected={selectedMaterial?.id === material.id}
              onClick={() => handleMaterialClick(material)}
            >
              <div className="material-card-content">
                <div className="material-emoji">{material.emoji}</div>
                <h3 className="material-name">{material.name}</h3>
                <p className="material-description">{material.description}</p>
              </div>
            </Card>
          ))}
        </div>
      </div>

      <div className="page-footer">
        {selectedMaterial && (
          <div className="selected-material-chip">
            선택된 재료: {selectedMaterial.emoji} {selectedMaterial.name}
          </div>
        )}
        <Button
          variant="primary"
          size="lg"
          disabled={!selectedMaterial}
          onClick={handleNext}
        >
          다음 →
        </Button>
      </div>
    </div>
  )
}
