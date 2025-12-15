/**
 * 작업 설정 플로우 상태 관리 (Zustand)
 */
import { create } from 'zustand'

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

interface Mode {
  id: string
  name: string
  icon: string
  description: string
  estimatedTime: number
  supportedMaterials: string[]
  parameters: any
}

interface JobParameters {
  materialSize: 'small' | 'medium' | 'large'
  repeatCount: number
  fryingParams?: {
    pieces: number
  }
  slicingParams?: {
    thickness: number
  }
}

interface JobConfigState {
  material: Material | null
  mode: Mode | null
  parameters: JobParameters | null
  estimatedTime: number

  setMaterial: (material: Material) => void
  setMode: (mode: Mode) => void
  setParameters: (parameters: JobParameters) => void
  setEstimatedTime: (time: number) => void
  reset: () => void
}

export const useJobConfigStore = create<JobConfigState>((set) => ({
  material: null,
  mode: null,
  parameters: null,
  estimatedTime: 0,

  setMaterial: (material) => set({ material }),
  setMode: (mode) => set({ mode }),
  setParameters: (parameters) => set({ parameters }),
  setEstimatedTime: (estimatedTime) => set({ estimatedTime }),
  reset: () => set({
    material: null,
    mode: null,
    parameters: null,
    estimatedTime: 0,
  }),
}))
