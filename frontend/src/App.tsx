import { BrowserRouter, Routes, Route } from 'react-router-dom'
import { Layout } from '@components/Layout'

// 페이지 임포트 (나중에 추가)
import Dashboard from '@pages/Dashboard'
import MaterialSelection from '@pages/MaterialSelection'
import ModeSelection from '@pages/ModeSelection'
import ParameterConfiguration from '@pages/ParameterConfiguration'
import JobConfirmation from '@pages/JobConfirmation'
import JobMonitoring from '@pages/JobMonitoring'
import JobCompletion from '@pages/JobCompletion'
import RobotStatus from '@pages/RobotStatus'
import PathConfiguration from '@pages/PathConfiguration'

function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Layout />}>
          {/* MCA-01: 메인 대시보드 */}
          <Route index element={<Dashboard />} />
          <Route path="dashboard" element={<Dashboard />} />

          {/* MCA-02: 작업 설정 플로우 */}
          <Route path="job/new/material" element={<MaterialSelection />} />
          <Route path="job/new/mode" element={<ModeSelection />} />
          <Route path="job/new/parameters" element={<ParameterConfiguration />} />
          <Route path="job/new/confirm" element={<JobConfirmation />} />

          {/* MCA-03: 작업 모니터링 */}
          <Route path="job/monitor/:jobId" element={<JobMonitoring />} />

          {/* MCA-04: 작업 완료 */}
          <Route path="job/complete/:jobId" element={<JobCompletion />} />

          {/* MCA-05: 로봇 상태 */}
          <Route path="robot/status" element={<RobotStatus />} />

          {/* MCA-06: 경로 설정 */}
          <Route path="path/editor" element={<PathConfiguration />} />
        </Route>
      </Routes>
    </BrowserRouter>
  )
}

export default App
