'use client'

import { useState, useEffect } from 'react'
import { useRouter } from 'next/navigation'
import Link from 'next/link'
import { Card } from '@/components/Card'
import { Button } from '@/components/Button'
import { ProgressIndicator } from '@/components/ProgressIndicator'

export default function JobConfirmation() {
  const router = useRouter()
  const [material, setMaterial] = useState<any>(null)
  const [mode, setMode] = useState<any>(null)
  const [parameters, setParameters] = useState<any>({})
  const [submitting, setSubmitting] = useState(false)

  useEffect(() => {
    const storedMaterial = localStorage.getItem('selectedMaterial')
    const storedMode = localStorage.getItem('selectedMode')
    const storedParameters = localStorage.getItem('jobParameters')

    if (!storedMaterial || !storedMode) {
      router.push('/job/material')
      return
    }

    setMaterial(JSON.parse(storedMaterial))
    setMode(JSON.parse(storedMode))
    setParameters(storedParameters ? JSON.parse(storedParameters) : {})
  }, [router])

  const handleConfirm = async () => {
    if (!material || !mode) return

    setSubmitting(true)
    try {
      const response = await fetch('/api/jobs', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          material: material.id,
          mode: mode.id,
          parameters: parameters,
          estimatedTime: mode.duration || 0,
        }),
      })

      const result = await response.json()

      if (result.success) {
        // Clear localStorage
        localStorage.removeItem('selectedMaterial')
        localStorage.removeItem('selectedMode')
        localStorage.removeItem('jobParameters')

        // Redirect to task monitor
        router.push('/tasks/monitor')
      } else {
        alert('작업 생성에 실패했습니다: ' + result.error)
      }
    } catch (error) {
      console.error('Failed to create job:', error)
      alert('작업 생성 중 오류가 발생했습니다.')
    } finally {
      setSubmitting(false)
    }
  }

  if (!material || !mode) {
    return null
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <Link href="/job/parameters">
          <Button variant="ghost">← 뒤로</Button>
        </Link>
        <h1 className="text-2xl font-bold text-gray-900">작업 확인</h1>
        <ProgressIndicator current={4} total={4} />
      </div>

      {/* Content */}
      <div className="space-y-6">
        <h2 className="text-xl font-semibold text-gray-900">
          다음 내용으로 작업을 시작하시겠어요?
        </h2>

        <Card variant="elevated">
          <div className="space-y-6">
            {/* Material */}
            <div>
              <h3 className="text-sm font-semibold text-gray-500 uppercase mb-2">
                재료
              </h3>
              <div className="flex items-center space-x-3">
                <span className="text-4xl">{material.emoji}</span>
                <div>
                  <div className="text-lg font-semibold text-gray-900">
                    {material.name}
                  </div>
                  <div className="text-sm text-gray-600">
                    {material.description}
                  </div>
                </div>
              </div>
            </div>

            <div className="border-t border-gray-200" />

            {/* Mode */}
            <div>
              <h3 className="text-sm font-semibold text-gray-500 uppercase mb-2">
                손질 모드
              </h3>
              <div className="flex items-center space-x-3">
                <span className="text-4xl">{mode.icon}</span>
                <div>
                  <div className="text-lg font-semibold text-gray-900">
                    {mode.name}
                  </div>
                  <div className="text-sm text-gray-600">
                    {mode.description}
                  </div>
                  <div className="text-sm text-gray-500 mt-1">
                    예상 시간: 약 {mode.duration || 0}초
                  </div>
                </div>
              </div>
            </div>

            {/* Parameters */}
            {Object.keys(parameters).length > 0 && (
              <>
                <div className="border-t border-gray-200" />
                <div>
                  <h3 className="text-sm font-semibold text-gray-500 uppercase mb-2">
                    작업 설정
                  </h3>
                  <div className="space-y-2">
                    {parameters.thickness && (
                      <div className="flex items-center justify-between">
                        <span className="text-sm text-gray-600">두께</span>
                        <span className="text-base font-semibold text-gray-900">
                          {parameters.thickness}mm
                        </span>
                      </div>
                    )}
                  </div>
                </div>
              </>
            )}
          </div>
        </Card>

        {/* Warning */}
        <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4">
          <div className="flex items-start space-x-3">
            <span className="text-2xl">⚠️</span>
            <div className="text-sm text-yellow-800">
              <p className="font-semibold mb-1">작업 시작 전 확인사항</p>
              <ul className="list-disc list-inside space-y-1">
                <li>로봇 작업 영역에 장애물이 없는지 확인하세요.</li>
                <li>재료가 올바르게 배치되었는지 확인하세요.</li>
                <li>비상 정지 버튼의 위치를 파악하세요.</li>
              </ul>
            </div>
          </div>
        </div>
      </div>

      {/* Footer */}
      <div className="flex space-x-4 pt-6 border-t border-gray-200">
        <Link href="/job/mode" className="flex-1">
          <Button variant="secondary" size="lg" fullWidth>
            수정하기
          </Button>
        </Link>
        <Button
          variant="primary"
          size="lg"
          onClick={handleConfirm}
          disabled={submitting}
          className="flex-1"
        >
          {submitting ? '작업 생성 중...' : '작업 시작 →'}
        </Button>
      </div>
    </div>
  )
}
