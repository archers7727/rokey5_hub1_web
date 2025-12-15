/**
 * 작업(Job) 타입 정의
 */

import { MaterialSize } from './material'
import { FryingModeParams, SlicingModeParams } from './mode'

export type JobStatus =
  | 'pending'
  | 'running'
  | 'paused'
  | 'stopped'
  | 'completed'
  | 'error'

export interface JobParameters {
  materialSize: MaterialSize
  repeatCount: number
  fryingParams?: FryingModeParams
  slicingParams?: SlicingModeParams
}

export interface Job {
  id: string
  material: string
  mode: string
  parameters: JobParameters
  status: JobStatus
  progress: number // 0-100
  currentStep: number
  totalSteps?: number
  createdAt: string
  startedAt: string | null
  completedAt: string | null
  estimatedTime: number // seconds
  actualTime: number // seconds
  speed?: number // 50-150%
}

export interface CreateJobRequest {
  material: string
  mode: string
  parameters: JobParameters
  estimatedTime: number
}

export interface JobMonitoringData {
  jobId: string
  status: JobStatus
  progress: number
  currentStep: number
  totalSteps: number
  stepDescription: string
  jointAngles: number[]
  tcpPosition: {
    x: number
    y: number
    z: number
  }
  speed: number
}
