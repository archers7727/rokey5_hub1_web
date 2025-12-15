/**
 * 경로(Path) 타입 정의
 */

export type PointType = 'pickup' | 'work' | 'place' | 'waypoint'

export interface PathPoint {
  id: string
  type: PointType
  x: number  // meters
  y: number  // meters
  z: number  // meters
  rx: number // degrees
  ry: number // degrees
  rz: number // degrees
}

export interface Path {
  id: string
  name: string
  isLocked: boolean
  points: PathPoint[]
  createdAt: string
  updatedAt: string
}

export interface PathValidationResult {
  valid: boolean
  error?: string
  message?: string
}
