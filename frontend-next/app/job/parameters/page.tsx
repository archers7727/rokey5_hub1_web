'use client'

import { useState, useEffect } from 'react'
import { useRouter } from 'next/navigation'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { ProgressIndicator } from '@/components/ProgressIndicator'

export const dynamic = 'force-dynamic'

interface Mode {
  id: string
  name: string
  icon: string
  description: string
  duration: number
}

const SLICING_THICKNESS_OPTIONS = [10, 20, 30, 40, 50] // mm 단위

export default function ParameterSelection() {
  const router = useRouter()
  const [material, setMaterial] = useState<any>(null)
  const [mode, setMode] = useState<Mode | null>(null)
  const [selectedThickness, setSelectedThickness] = useState<number | null>(null)

  useEffect(() => {
    // Load material and mode from localStorage
    const storedMaterial = localStorage.getItem('selectedMaterial')
    const storedMode = localStorage.getItem('selectedMode')

    if (!storedMaterial || !storedMode) {
      router.push('/job/material')
      return
    }

    setMaterial(JSON.parse(storedMaterial))
    setMode(JSON.parse(storedMode))
  }, [router])

  const handleThicknessClick = (thickness: number) => {
    setSelectedThickness(thickness)
  }

  const handleNext = () => {
    if (selectedThickness && mode) {
      // Save parameters to localStorage
      const parameters = {
        thickness: selectedThickness
      }

      localStorage.setItem('jobParameters', JSON.stringify(parameters))
      router.push('/job/confirm')
    }
  }

  if (!mode || !material) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-lg text-gray-600">로딩 중...</div>
      </div>
    )
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <Link href="/job/mode">
          <Button variant="ghost">← 뒤로</Button>
        </Link>
        <h1 className="text-2xl font-bold text-gray-900">작업 설정</h1>
        <ProgressIndicator current={3} total={4} />
      </div>

      {/* Selected Items Badge */}
      <div className="flex items-center space-x-4">
        <div className="flex items-center space-x-2 bg-gray-100 px-4 py-2 rounded-lg">
          <span className="text-2xl">{material.emoji}</span>
          <span className="text-gray-700 text-sm">{material.name}</span>
        </div>
        <div className="flex items-center space-x-2 bg-gray-100 px-4 py-2 rounded-lg">
          <span className="text-2xl">{mode.icon}</span>
          <span className="text-gray-700 text-sm">{mode.name}</span>
        </div>
      </div>

      {/* Content */}
      <div className="space-y-6">
        {mode.id === 'slicing' && (
          <>
            <h2 className="text-xl font-semibold text-gray-900">
              얼마나 얇게 썰어드릴까요?
            </h2>
            <p className="text-sm text-gray-600">
              원하시는 두께를 선택해주세요
            </p>

            <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-5 gap-4">
              {SLICING_THICKNESS_OPTIONS.map((thickness) => (
                <Card
                  key={thickness}
                  selectable
                  selected={selectedThickness === thickness}
                  onClick={() => handleThicknessClick(thickness)}
                  className="cursor-pointer"
                >
                  <div className="text-center py-4">
                    <div className="text-3xl font-bold text-gray-900">
                      {thickness}
                    </div>
                    <div className="text-sm text-gray-500 mt-1">mm</div>
                  </div>
                </Card>
              ))}
            </div>

            {/* Visual Reference */}
            {selectedThickness && (
              <Card variant="outlined" className="bg-blue-50 border-blue-200">
                <div className="flex items-center space-x-3">
                  <span className="text-2xl">📏</span>
                  <div>
                    <div className="text-sm font-semibold text-blue-900">
                      선택된 두께: {selectedThickness}mm
                    </div>
                    <div className="text-xs text-blue-700 mt-1">
                      {selectedThickness === 10 && '매우 얇게 (샐러드용)'}
                      {selectedThickness === 20 && '얇게 (볶음용)'}
                      {selectedThickness === 30 && '보통 (일반 조리용)'}
                      {selectedThickness === 40 && '두껍게 (구이용)'}
                      {selectedThickness === 50 && '매우 두껍게 (스테이크용)'}
                    </div>
                  </div>
                </div>
              </Card>
            )}
          </>
        )}

        {mode.id === 'frying' && (
          <>
            <h2 className="text-xl font-semibold text-gray-900">
              튀김 설정
            </h2>
            <Card variant="outlined">
              <div className="text-center py-8">
                <p className="text-gray-600">
                  프레스 모드는 기본 설정으로 진행됩니다
                </p>
                <p className="text-sm text-gray-500 mt-2">
                  -
                </p>
              </div>
            </Card>
          </>
        )}

        {mode.id === 'dicing' && (
          <>
            <h2 className="text-xl font-semibold text-gray-900">
              깍둑썰기 설정
            </h2>
            <Card variant="outlined">
              <div className="text-center py-8">
                <p className="text-gray-600">
                  깍둑썰기는 기본 설정으로 진행됩니다
                </p>
                <p className="text-sm text-gray-500 mt-2">
                  정육면체 모양으로 썰어드립니다 🎲
                </p>
              </div>
            </Card>
          </>
        )}
      </div>

      {/* Footer */}
      <div className="pt-6 border-t border-gray-200">
        <Button
          variant="primary"
          size="lg"
          disabled={mode.id === 'slicing' && !selectedThickness}
          onClick={handleNext}
          fullWidth
        >
          다음 →
        </Button>
      </div>
    </div>
  )
}
