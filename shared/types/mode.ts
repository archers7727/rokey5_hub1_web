/**
 * 손질 모드(Mode) 타입 정의
 */

export type ModeType = 'frying' | 'slicing'

export interface ModeParameter {
  type: 'select' | 'slider' | 'number'
  label: string
  min?: number
  max?: number
  step?: number
  default: number | string
  unit?: string
  options?: (number | string)[]
}

export interface Mode {
  id: string
  name: string
  icon: string
  description: string
  estimatedTime: number // seconds
  supportedMaterials: string[]
  parameters: {
    [key: string]: ModeParameter
  }
}

export interface FryingModeParams {
  pieces: number // 2, 4, 6, 8
}

export interface SlicingModeParams {
  thickness: number // mm
}
