/**
 * 로봇 상태 전역 관리 (Zustand)
 */
import { create } from 'zustand'

type RobotStatus = 'idle' | 'working' | 'emergency_stop' | 'user_stop' | 'error'

interface RobotState {
  status: RobotStatus
  model: string
  isConnected: boolean
  jointAngles: number[]
  jointLoads: number[]
  tcpPosition: {
    x: number
    y: number
    z: number
    rx: number
    ry: number
    rz: number
  }
  speed: number

  setStatus: (status: RobotStatus) => void
  setJointAngles: (angles: number[]) => void
  setTcpPosition: (position: any) => void
  setSpeed: (speed: number) => void
  updateRobotState: (state: Partial<RobotState>) => void
}

export const useRobotStore = create<RobotState>((set) => ({
  status: 'idle',
  model: 'M1013',
  isConnected: false,
  jointAngles: [0, 0, 90, 0, 90, 0],
  jointLoads: [0, 0, 0, 0, 0, 0],
  tcpPosition: {
    x: 0.450,
    y: 0.000,
    z: 0.680,
    rx: 0.0,
    ry: 0.0,
    rz: 0.0,
  },
  speed: 0.0,

  setStatus: (status) => set({ status }),
  setJointAngles: (jointAngles) => set({ jointAngles }),
  setTcpPosition: (tcpPosition) => set({ tcpPosition }),
  setSpeed: (speed) => set({ speed }),
  updateRobotState: (state) => set(state),
}))
