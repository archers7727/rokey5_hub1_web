/**
 * 로봇(Robot) 타입 정의
 */

export type RobotStatus =
  | 'idle'
  | 'working'
  | 'emergency_stop'
  | 'user_stop'
  | 'error'

export interface RobotState {
  status: RobotStatus
  model: string
  isConnected: boolean
  jointAngles: number[] // 6개 관절 각도 (degrees)
  jointLoads: number[]  // 6개 관절 부하 (0-100%)
  tcpPosition: TCPPosition
  speed: number // m/s
}

export interface TCPPosition {
  x: number  // meters
  y: number  // meters
  z: number  // meters
  rx: number // degrees
  ry: number // degrees
  rz: number // degrees
}

export interface JointInfo {
  jointNumber: number
  angle: number // degrees
  load: number  // 0-100%
  temperature?: number // celsius
}
