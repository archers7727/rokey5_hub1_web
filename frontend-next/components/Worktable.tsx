/**
 * 작업대(주방) 3D 모델
 */
'use client'

export function Worktable() {
  return (
    <group position={[0, 0, 0]}>
      {/* 바닥 */}
      <mesh position={[0, -0.01, 0]} receiveShadow>
        <boxGeometry args={[3, 0.02, 3]} />
        <meshStandardMaterial color="#bdc3c7" roughness={0.8} />
      </mesh>

      {/* 작업대 상판 - 스테인리스 느낌 */}
      <mesh position={[0.4, 0.4, 0]} receiveShadow>
        <boxGeometry args={[1.2, 0.03, 0.8]} />
        <meshStandardMaterial color="#c0c0c0" roughness={0.2} metalness={0.8} />
      </mesh>

      {/* 작업대 다리 1 */}
      <mesh position={[1.0, 0.2, 0.4]}>
        <boxGeometry args={[0.05, 0.4, 0.05]} />
        <meshStandardMaterial color="#7f8c8d" />
      </mesh>

      {/* 작업대 다리 2 */}
      <mesh position={[1.0, 0.2, -0.4]}>
        <boxGeometry args={[0.05, 0.4, 0.05]} />
        <meshStandardMaterial color="#7f8c8d" />
      </mesh>

      {/* 작업대 다리 3 */}
      <mesh position={[-0.2, 0.2, 0.4]}>
        <boxGeometry args={[0.05, 0.4, 0.05]} />
        <meshStandardMaterial color="#7f8c8d" />
      </mesh>

      {/* 작업대 다리 4 */}
      <mesh position={[-0.2, 0.2, -0.4]}>
        <boxGeometry args={[0.05, 0.4, 0.05]} />
        <meshStandardMaterial color="#7f8c8d" />
      </mesh>

      {/* 주방 백스플래시 - 타일 느낌 */}
      <mesh position={[1.05, 0.6, 0]}>
        <boxGeometry args={[0.02, 0.5, 0.8]} />
        <meshStandardMaterial color="#ffffff" roughness={0.3} metalness={0.2} />
      </mesh>

      {/* 싱크대 */}
      <mesh position={[0.7, 0.42, 0.2]}>
        <boxGeometry args={[0.3, 0.08, 0.25]} />
        <meshStandardMaterial color="#a8a8a8" metalness={0.9} roughness={0.1} />
      </mesh>

      {/* === 식재료 === */}

      {/* 도마 */}
      <mesh position={[0.2, 0.415, -0.15]} rotation={[0, 0.3, 0]}>
        <boxGeometry args={[0.25, 0.015, 0.35]} />
        <meshStandardMaterial color="#d4a574" roughness={0.6} />
      </mesh>

      {/* 당근 1 */}
      <group position={[0.1, 0.43, -0.1]}>
        <mesh rotation={[0, 0, Math.PI / 6]}>
          <cylinderGeometry args={[0.015, 0.02, 0.12, 16]} />
          <meshStandardMaterial color="#ff8c42" roughness={0.7} />
        </mesh>
        {/* 당근 잎 */}
        <mesh position={[0, 0.065, 0]}>
          <coneGeometry args={[0.02, 0.03, 8]} />
          <meshStandardMaterial color="#2ecc71" roughness={0.6} />
        </mesh>
      </group>

      {/* 당근 2 */}
      <group position={[0.15, 0.43, -0.05]}>
        <mesh rotation={[0, 0, -Math.PI / 8]}>
          <cylinderGeometry args={[0.015, 0.02, 0.1, 16]} />
          <meshStandardMaterial color="#ff8c42" roughness={0.7} />
        </mesh>
        <mesh position={[0, 0.055, 0]}>
          <coneGeometry args={[0.02, 0.03, 8]} />
          <meshStandardMaterial color="#2ecc71" roughness={0.6} />
        </mesh>
      </group>

      {/* 오이 */}
      <mesh position={[0.25, 0.43, -0.2]} rotation={[0, 0.5, Math.PI / 2]}>
        <cylinderGeometry args={[0.02, 0.02, 0.15, 16]} />
        <meshStandardMaterial color="#2ecc71" roughness={0.5} />
      </mesh>

      {/* 토마토 */}
      <mesh position={[0.3, 0.44, -0.15]}>
        <sphereGeometry args={[0.035, 16, 16]} />
        <meshStandardMaterial color="#e74c3c" roughness={0.4} />
      </mesh>
      {/* 토마토 꼭지 */}
      <mesh position={[0.3, 0.475, -0.15]}>
        <coneGeometry args={[0.015, 0.02, 5]} />
        <meshStandardMaterial color="#27ae60" roughness={0.6} />
      </mesh>

      {/* 양파 */}
      <mesh position={[0.05, 0.435, -0.25]}>
        <sphereGeometry args={[0.04, 16, 16]} />
        <meshStandardMaterial color="#dda15e" roughness={0.6} />
      </mesh>
      {/* 양파 뿌리 */}
      <mesh position={[0.05, 0.395, -0.25]}>
        <cylinderGeometry args={[0.01, 0.015, 0.02, 8]} />
        <meshStandardMaterial color="#8b4513" roughness={0.8} />
      </mesh>

      {/* 칼 */}
      <group position={[0.35, 0.42, -0.05]} rotation={[0, Math.PI / 4, 0]}>
        {/* 칼날 */}
        <mesh position={[0, 0.005, 0]}>
          <boxGeometry args={[0.015, 0.01, 0.15]} />
          <meshStandardMaterial color="#c0c0c0" metalness={0.95} roughness={0.05} />
        </mesh>
        {/* 손잡이 */}
        <mesh position={[0, 0.005, -0.09]}>
          <boxGeometry args={[0.02, 0.015, 0.08]} />
          <meshStandardMaterial color="#2c3e50" roughness={0.6} />
        </mesh>
      </group>

      {/* 그리드 라인 (작업 영역 표시) */}
      <gridHelper args={[2, 20, '#34495e', '#7f8c8d']} position={[0, 0, 0]} />
    </group>
  )
}
