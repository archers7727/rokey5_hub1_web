/**
 * 로봇팔 3D 뷰어 컴포넌트
 * Three.js와 React Three Fiber를 사용한 3D 시각화
 */
import { Suspense } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment } from '@react-three/drei'
import { DoosanM0609 } from './DoosanM0609'
import { Worktable } from './Worktable'

interface RobotArm3DViewerProps {
  jointAngles: number[]
  progress?: number
}

function Scene({ jointAngles }: { jointAngles: number[] }) {
  return (
    <>
      {/* 카메라 */}
      <PerspectiveCamera makeDefault position={[2, 1.5, 2]} fov={50} />

      {/* 조명 */}
      <ambientLight intensity={0.5} />
      <directionalLight
        position={[5, 5, 5]}
        intensity={1}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
      />
      <directionalLight position={[-5, 3, -5]} intensity={0.3} />
      <pointLight position={[0, 2, 0]} intensity={0.5} />

      {/* 환경 */}
      <Environment preset="studio" />

      {/* 작업대 */}
      <Worktable />

      {/* 로봇팔 */}
      <DoosanM0609 jointAngles={jointAngles} />

      {/* 컨트롤 */}
      <OrbitControls
        enablePan={true}
        enableZoom={true}
        enableRotate={true}
        minDistance={1}
        maxDistance={5}
        maxPolarAngle={Math.PI / 2}
      />
    </>
  )
}

function LoadingFallback() {
  return (
    <mesh>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color="#3498db" />
    </mesh>
  )
}

export function RobotArm3DViewer({ jointAngles, progress = 0 }: RobotArm3DViewerProps) {
  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <Canvas
        shadows
        style={{
          width: '100%',
          height: '100%',
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        }}
      >
        <Suspense fallback={<LoadingFallback />}>
          <Scene jointAngles={jointAngles} />
        </Suspense>
      </Canvas>

      {/* 진행률 오버레이 */}
      <div
        style={{
          position: 'absolute',
          top: '16px',
          right: '16px',
          background: 'rgba(0, 0, 0, 0.7)',
          color: 'white',
          padding: '12px 20px',
          borderRadius: '8px',
          fontWeight: 'bold',
          fontSize: '18px',
          backdropFilter: 'blur(10px)',
        }}
      >
        {Math.round(progress)}%
      </div>

      {/* 컨트롤 안내 */}
      <div
        style={{
          position: 'absolute',
          bottom: '16px',
          left: '16px',
          background: 'rgba(0, 0, 0, 0.6)',
          color: 'white',
          padding: '8px 12px',
          borderRadius: '6px',
          fontSize: '12px',
          backdropFilter: 'blur(10px)',
        }}
      >
        🖱️ 마우스로 회전 | 🔍 휠로 줌 | 👆 드래그로 이동
      </div>
    </div>
  )
}
