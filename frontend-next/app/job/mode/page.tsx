'use client'

import { useState, useEffect } from 'react'
import { useRouter } from 'next/navigation'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { ProgressIndicator } from '@/components/ProgressIndicator'

interface Mode {
  id: string
  name: string
  icon: string
  description: string
  duration: number
  compatible_materials: string[]
}

export default function ModeSelection() {
  const router = useRouter()
  const [material, setMaterial] = useState<any>(null)
  const [modes, setModes] = useState<Mode[]>([])
  const [selectedMode, setSelectedMode] = useState<Mode | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    // Load material from localStorage
    const storedMaterial = localStorage.getItem('selectedMaterial')
    if (!storedMaterial) {
      router.push('/job/material')
      return
    }

    const parsedMaterial = JSON.parse(storedMaterial)
    setMaterial(parsedMaterial)
    loadModes(parsedMaterial.id)
  }, [router])

  const loadModes = async (materialId: string) => {
    try {
      const response = await fetch(`/api/modes?material=${materialId}`)
      const result = await response.json()

      if (result.success) {
        setModes(result.data)
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
      localStorage.setItem('selectedMode', JSON.stringify(selectedMode))
      router.push('/job/confirm')
    }
  }

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-lg text-gray-600">모드 목록을 불러오는 중...</div>
      </div>
    )
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <Link href="/job/material">
          <Button variant="ghost">← 뒤로</Button>
        </Link>
        <h1 className="text-2xl font-bold text-gray-900">손질 모드 선택</h1>
        <ProgressIndicator current={2} total={4} />
      </div>

      {/* Selected Material Badge */}
      {material && (
        <div className="flex items-center space-x-2 bg-gray-100 px-4 py-2 rounded-lg w-fit">
          <span className="text-2xl">{material.emoji}</span>
          <span className="text-gray-700">
            선택한 재료: <strong>{material.name}</strong>
          </span>
        </div>
      )}

      {/* Content */}
      <div className="space-y-6">
        <h2 className="text-xl font-semibold text-gray-900">
          어떻게 손질하시겠어요?
        </h2>

        <div className="space-y-4">
          {modes.map((mode) => (
            <Card
              key={mode.id}
              selectable
              selected={selectedMode?.id === mode.id}
              onClick={() => handleModeClick(mode)}
            >
              <div className="flex items-center space-x-4">
                <div className="text-5xl">{mode.icon}</div>
                <div className="flex-1">
                  <h3 className="text-lg font-semibold text-gray-900">
                    {mode.name}
                  </h3>
                  <p className="text-sm text-gray-600 mt-1">
                    {mode.description}
                  </p>
                  <div className="text-sm text-gray-500 mt-2">
                    예상 시간: 약 {Math.floor(mode.duration / 60)}분
                  </div>
                </div>
              </div>
            </Card>
          ))}
        </div>

        {modes.length === 0 && (
          <div className="text-center py-12">
            <p className="text-gray-500">
              이 재료에 사용 가능한 모드가 없습니다.
            </p>
            <p className="text-sm text-gray-400 mt-2">
              Supabase에서 modes 데이터를 추가해주세요.
            </p>
          </div>
        )}
      </div>

      {/* Footer */}
      <div className="pt-6 border-t border-gray-200">
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
