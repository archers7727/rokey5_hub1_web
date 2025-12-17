/**
 * 작업대(주방) 3D 모델
 */
export function Worktable() {
  return (
    <group position={[0, 0, 0]}>
      {/* 바닥 */}
      <mesh position={[0, -0.01, 0]} receiveShadow>
        <boxGeometry args={[3, 0.02, 3]} />
        <meshStandardMaterial color="#bdc3c7" roughness={0.8} />
      </mesh>

      {/* 작업대 상판 */}
      <mesh position={[0.4, 0.4, 0]} receiveShadow>
        <boxGeometry args={[1.2, 0.03, 0.8]} />
        <meshStandardMaterial color="#ecf0f1" roughness={0.3} metalness={0.4} />
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

      {/* 주방 백스플래시 */}
      <mesh position={[1.05, 0.6, 0]}>
        <boxGeometry args={[0.02, 0.5, 0.8]} />
        <meshStandardMaterial color="#3498db" roughness={0.2} metalness={0.6} />
      </mesh>

      {/* 싱크대 (장식용) */}
      <mesh position={[0.7, 0.42, 0.2]}>
        <boxGeometry args={[0.3, 0.08, 0.25]} />
        <meshStandardMaterial color="#95a5a6" metalness={0.8} roughness={0.2} />
      </mesh>

      {/* 재료 배치 영역 표시 */}
      <mesh position={[0.1, 0.415, -0.2]}>
        <boxGeometry args={[0.25, 0.005, 0.25]} />
        <meshStandardMaterial color="#2ecc71" transparent opacity={0.3} />
      </mesh>

      {/* 그리드 라인 (작업 영역 표시) */}
      <gridHelper args={[2, 20, '#34495e', '#7f8c8d']} position={[0, 0, 0]} />
    </group>
  )
}
