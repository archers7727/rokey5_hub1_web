/**
 * API 클라이언트
 */
import axios from 'axios'

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || '/api'

export const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
})

// 요청 인터셉터
apiClient.interceptors.request.use(
  (config) => {
    // 향후 인증 토큰 추가
    return config
  },
  (error) => {
    return Promise.reject(error)
  }
)

// 응답 인터셉터
apiClient.interceptors.response.use(
  (response) => {
    return response.data
  },
  (error) => {
    console.error('API Error:', error)
    return Promise.reject(error)
  }
)

// API 함수들
export const materialsApi = {
  getAll: () => apiClient.get('/materials'),
  getById: (id: string) => apiClient.get(`/materials/${id}`),
}

export const modesApi = {
  getAll: (material?: string) =>
    apiClient.get('/modes', { params: { material } }),
  getById: (id: string) => apiClient.get(`/modes/${id}`),
}

export const jobsApi = {
  create: (data: any) => apiClient.post('/jobs', data),
  getById: (id: string) => apiClient.get(`/jobs/${id}`),
  getAll: (limit = 10) => apiClient.get('/jobs', { params: { limit } }),
  start: (id: string) => apiClient.post(`/jobs/${id}/start`),
  pause: (id: string) => apiClient.post(`/jobs/${id}/pause`),
  resume: (id: string) => apiClient.post(`/jobs/${id}/resume`),
  stop: (id: string) => apiClient.post(`/jobs/${id}/stop`),
  setSpeed: (id: string, speed: number) =>
    apiClient.post(`/jobs/${id}/speed`, { speed }),
}

export const robotApi = {
  getStatus: () => apiClient.get('/robot/status'),
  emergencyStop: () => apiClient.post('/robot/emergency-stop'),
  releaseEmergency: () => apiClient.post('/robot/release-emergency'),
  resume: () => apiClient.post('/robot/resume'),
  stop: () => apiClient.post('/robot/stop'),
}

export const dashboardApi = {
  getData: () => apiClient.get('/dashboard'),
}

export const pathsApi = {
  getAll: () => apiClient.get('/paths'),
  getById: (id: string) => apiClient.get(`/paths/${id}`),
  create: (data: any) => apiClient.post('/paths', data),
  update: (id: string, data: any) => apiClient.put(`/paths/${id}`, data),
  delete: (id: string) => apiClient.delete(`/paths/${id}`),
  validate: (id: string) => apiClient.post(`/paths/${id}/validate`),
}
