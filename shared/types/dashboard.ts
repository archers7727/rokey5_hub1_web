/**
 * 대시보드(Dashboard) 타입 정의
 */

export interface TodayStats {
  jobCount: number
  totalTime: number // seconds
  successRate: number // 0-100
}

export interface RecentJob {
  id: string
  material: string
  mode: string
  completedAt: string
  duration: number // seconds
  status: 'completed' | 'error'
}

export interface DashboardData {
  todayStats: TodayStats
  recentJobs: RecentJob[]
}
