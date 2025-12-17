/**
 * 두산 M0609 로봇팔 3D 모델
 * 6축 로봇팔을 기하학적 형태로 표현
 */
'use client'

import { useRef, useEffect } from 'react'
import { Group } from 'three'

interface DoosanM0609Props {
  jointAngles: number[] // 6개의 관절 각도 (degrees)
}

export function DoosanM0609({ jointAngles }: DoosanM0609Props) {
  const baseRef = useRef<Group>(null)
  const joint1Ref = useRef<Group>(null)
  const joint2Ref = useRef<Group>(null)
  const joint3Ref = useRef<Group>(null)
  const joint4Ref = useRef<Group>(null)
  const joint5Ref = useRef<Group>(null)
  const joint6Ref = useRef<Group>(null)

  // 각도를 라디안으로 변환
  const toRad = (deg: number) => (deg * Math.PI) / 180

  // 관절 각도 업데이트
  useEffect(() => {
    if (joint1Ref.current) joint1Ref.current.rotation.y = toRad(jointAngles[0] || 0)
    if (joint2Ref.current) joint2Ref.current.rotation.z = toRad(jointAngles[1] || 0)
    if (joint3Ref.current) joint3Ref.current.rotation.z = toRad(jointAngles[2] || 0)
    if (joint4Ref.current) joint4Ref.current.rotation.x = toRad(jointAngles[3] || 0)
    if (joint5Ref.current) joint5Ref.current.rotation.z = toRad(jointAngles[4] || 0)
    if (joint6Ref.current) joint6Ref.current.rotation.x = toRad(jointAngles[5] || 0)
  }, [jointAngles])

  return (
    <group ref={baseRef} position={[-0.2, 0, 0.1]} rotation={[0, -Math.PI / 6, 0]}>
      {/* 베이스 */}
      <mesh position={[0, 0.05, 0]}>
        <cylinderGeometry args={[0.15, 0.18, 0.1, 32]} />
        <meshStandardMaterial
          color="#e8e8e8"
          metalness={0.3}
          roughness={0.4}
          emissive="#ffffff"
          emissiveIntensity={0.1}
        />
      </mesh>
      {/* 베이스 윤곽선 */}
      <mesh position={[0, 0.05, 0]}>
        <cylinderGeometry args={[0.151, 0.181, 0.101, 32]} />
        <meshBasicMaterial color="#888888" wireframe />
      </mesh>

      {/* Joint 1 - 회전 베이스 */}
      <group ref={joint1Ref} position={[0, 0.1, 0]}>
        <mesh position={[0, 0.08, 0]}>
          <cylinderGeometry args={[0.12, 0.12, 0.16, 32]} />
          <meshStandardMaterial
            color="#f5f5f5"
            metalness={0.3}
            roughness={0.4}
            emissive="#ffffff"
            emissiveIntensity={0.1}
          />
        </mesh>
        {/* Joint 1 윤곽선 */}
        <mesh position={[0, 0.08, 0]}>
          <cylinderGeometry args={[0.121, 0.121, 0.161, 32]} />
          <meshBasicMaterial color="#888888" wireframe />
        </mesh>

        {/* Joint 2 - 첫 번째 링크 (회색) */}
        <group ref={joint2Ref} position={[0, 0.16, 0]}>
          {/* 숄더 */}
          <mesh position={[0, 0, 0]}>
            <boxGeometry args={[0.12, 0.1, 0.12]} />
            <meshStandardMaterial
              color="#f5f5f5"
              metalness={0.3}
              roughness={0.4}
              emissive="#ffffff"
              emissiveIntensity={0.1}
            />
          </mesh>
          {/* 숄더 윤곽선 */}
          <mesh position={[0, 0, 0]}>
            <boxGeometry args={[0.121, 0.101, 0.121]} />
            <meshBasicMaterial color="#888888" wireframe />
          </mesh>

          {/* 상완 링크 (회색) */}
          <mesh position={[0, 0.15, 0]}>
            <boxGeometry args={[0.08, 0.3, 0.08]} />
            <meshStandardMaterial
              color="#b0b0b0"
              metalness={0.4}
              roughness={0.4}
            />
          </mesh>
          {/* 상완 링크 윤곽선 */}
          <mesh position={[0, 0.15, 0]}>
            <boxGeometry args={[0.081, 0.301, 0.081]} />
            <meshBasicMaterial color="#666666" wireframe />
          </mesh>

          {/* Joint 3 - 두 번째 링크 (엘보우, 회색) */}
          <group ref={joint3Ref} position={[0, 0.3, 0]}>
            <mesh position={[0, 0, 0]}>
              <sphereGeometry args={[0.06, 32, 32]} />
              <meshStandardMaterial
                color="#f5f5f5"
                metalness={0.3}
                roughness={0.4}
                emissive="#ffffff"
                emissiveIntensity={0.1}
              />
            </mesh>
            {/* 엘보우 윤곽선 */}
            <mesh position={[0, 0, 0]}>
              <sphereGeometry args={[0.061, 32, 32]} />
              <meshBasicMaterial color="#888888" wireframe />
            </mesh>

            {/* 전완 링크 (회색) */}
            <mesh position={[0, 0.125, 0]}>
              <boxGeometry args={[0.06, 0.25, 0.06]} />
              <meshStandardMaterial
                color="#b0b0b0"
                metalness={0.4}
                roughness={0.4}
              />
            </mesh>
            {/* 전완 링크 윤곽선 */}
            <mesh position={[0, 0.125, 0]}>
              <boxGeometry args={[0.061, 0.251, 0.061]} />
              <meshBasicMaterial color="#666666" wireframe />
            </mesh>

            {/* Joint 4 - 손목 회전 (하얀색) */}
            <group ref={joint4Ref} position={[0, 0.25, 0]}>
              <mesh position={[0, 0, 0]}>
                <cylinderGeometry args={[0.04, 0.04, 0.08, 32]} />
                <meshStandardMaterial
                  color="#f5f5f5"
                  metalness={0.3}
                  roughness={0.4}
                  emissive="#ffffff"
                  emissiveIntensity={0.1}
                />
              </mesh>
              {/* Joint 4 윤곽선 */}
              <mesh position={[0, 0, 0]}>
                <cylinderGeometry args={[0.041, 0.041, 0.081, 32]} />
                <meshBasicMaterial color="#888888" wireframe />
              </mesh>

              {/* Joint 5 - 손목 벤드 (하얀색) */}
              <group ref={joint5Ref} position={[0, 0.04, 0]}>
                <mesh position={[0, 0, 0]}>
                  <sphereGeometry args={[0.035, 32, 32]} />
                  <meshStandardMaterial
                    color="#f5f5f5"
                    metalness={0.3}
                    roughness={0.4}
                    emissive="#ffffff"
                    emissiveIntensity={0.1}
                  />
                </mesh>
                {/* Joint 5 윤곽선 */}
                <mesh position={[0, 0, 0]}>
                  <sphereGeometry args={[0.036, 32, 32]} />
                  <meshBasicMaterial color="#888888" wireframe />
                </mesh>

                {/* Joint 6 - 플랜지 (하얀색) */}
                <group ref={joint6Ref} position={[0, 0.04, 0]}>
                  <mesh position={[0, 0, 0]}>
                    <cylinderGeometry args={[0.05, 0.05, 0.02, 32]} />
                    <meshStandardMaterial
                      color="#e8e8e8"
                      metalness={0.3}
                      roughness={0.4}
                      emissive="#ffffff"
                      emissiveIntensity={0.1}
                    />
                  </mesh>
                  {/* Joint 6 윤곽선 */}
                  <mesh position={[0, 0, 0]}>
                    <cylinderGeometry args={[0.051, 0.051, 0.021, 32]} />
                    <meshBasicMaterial color="#888888" wireframe />
                  </mesh>

                  {/* 엔드 이펙터 (그리퍼) - 하얀색 */}
                  <group position={[0, 0.01, 0]}>
                    {/* 그리퍼 베이스 */}
                    <mesh position={[0, 0.02, 0]}>
                      <cylinderGeometry args={[0.03, 0.04, 0.04, 32]} />
                      <meshStandardMaterial
                        color="#f5f5f5"
                        metalness={0.3}
                        roughness={0.4}
                        emissive="#ffffff"
                        emissiveIntensity={0.1}
                      />
                    </mesh>
                    {/* 그리퍼 베이스 윤곽선 */}
                    <mesh position={[0, 0.02, 0]}>
                      <cylinderGeometry args={[0.031, 0.041, 0.041, 32]} />
                      <meshBasicMaterial color="#888888" wireframe />
                    </mesh>

                    {/* 그리퍼 핑거 1 */}
                    <mesh position={[0.025, 0.05, 0]}>
                      <boxGeometry args={[0.015, 0.06, 0.02]} />
                      <meshStandardMaterial
                        color="#f5f5f5"
                        metalness={0.3}
                        roughness={0.4}
                        emissive="#ffffff"
                        emissiveIntensity={0.1}
                      />
                    </mesh>
                    {/* 그리퍼 핑거 1 윤곽선 */}
                    <mesh position={[0.025, 0.05, 0]}>
                      <boxGeometry args={[0.016, 0.061, 0.021]} />
                      <meshBasicMaterial color="#888888" wireframe />
                    </mesh>

                    {/* 그리퍼 핑거 2 */}
                    <mesh position={[-0.025, 0.05, 0]}>
                      <boxGeometry args={[0.015, 0.06, 0.02]} />
                      <meshStandardMaterial
                        color="#f5f5f5"
                        metalness={0.3}
                        roughness={0.4}
                        emissive="#ffffff"
                        emissiveIntensity={0.1}
                      />
                    </mesh>
                    {/* 그리퍼 핑거 2 윤곽선 */}
                    <mesh position={[-0.025, 0.05, 0]}>
                      <boxGeometry args={[0.016, 0.061, 0.021]} />
                      <meshBasicMaterial color="#888888" wireframe />
                    </mesh>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  )
}
