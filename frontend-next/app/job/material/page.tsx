'use client'

import { useState, useEffect } from 'react'
import { useRouter } from 'next/navigation'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { ProgressIndicator } from '@/components/ProgressIndicator'

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
  const router = useRouter()
  const [materials, setMaterials] = useState<Material[]>([])
  const [selectedMaterial, setSelectedMaterial] = useState<Material | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    loadMaterials()
  }, [])

  const loadMaterials = async () => {
    try {
      const response = await fetch('/api/materials')
      const result = await response.json()

      if (result.success) {
        setMaterials(result.data)
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
      // Store in localStorage for now (later we'll use Zustand)
      localStorage.setItem('selectedMaterial', JSON.stringify(selectedMaterial))
      router.push('/job/mode')
    }
  }

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-lg text-gray-600">재료 목록을 불러오는 중...</div>
      </div>
    )
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <Link href="/dashboard">
          <Button variant="ghost">← 뒤로</Button>
        </Link>
        <h1 className="text-2xl font-bold text-gray-900">재료 선택</h1>
        <ProgressIndicator current={1} total={4} />
      </div>

      {/* Content */}
      <div className="space-y-6">
        <h2 className="text-xl font-semibold text-gray-900">
          어떤 재료를 손질하시겠어요?
        </h2>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {materials.map((material) => (
            <Card
              key={material.id}
              selectable
              selected={selectedMaterial?.id === material.id}
              onClick={() => handleMaterialClick(material)}
            >
              <div className="text-center space-y-3">
                <div className="text-6xl">{material.emoji}</div>
                <h3 className="text-lg font-semibold text-gray-900">
                  {material.name}
                </h3>
                <p className="text-sm text-gray-600">{material.description}</p>
              </div>
            </Card>
          ))}
        </div>

        {materials.length === 0 && (
          <div className="text-center py-12">
            <p className="text-gray-500">사용 가능한 재료가 없습니다.</p>
            <p className="text-sm text-gray-400 mt-2">
              Supabase에서 materials 데이터를 추가해주세요.
            </p>
          </div>
        )}
      </div>

      {/* Footer */}
      <div className="flex items-center justify-between pt-6 border-t border-gray-200">
        {selectedMaterial ? (
          <div className="flex items-center space-x-2 bg-blue-50 text-blue-700 px-4 py-2 rounded-lg">
            <span className="text-2xl">{selectedMaterial.emoji}</span>
            <span className="font-medium">
              선택된 재료: {selectedMaterial.name}
            </span>
          </div>
        ) : (
          <div />
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
