import { Canvas } from '@react-three/fiber'
import { Grid, OrbitControls, PerspectiveCamera, Text } from '@react-three/drei'
import { useMemo } from 'react'
import * as THREE from 'three'
import type { DemoPayload, TelemetryFrame } from './types'
import { makeTransform } from './sceneTransform'

function Drone({ frame, xy }: { frame: TelemetryFrame; xy: (lat: number, lon: number) => { x: number; z: number } }) {
  const p = xy(frame.lat_deg, frame.lon_deg)
  // Altitude in the twin is relative to launch; clamp the visual scale.
  const y = Math.max(0.4, Math.min(frame.alt_m / 10, 8))
  const rotor = 1.5
  const arms: [number, number][] = [[rotor, rotor], [rotor, -rotor], [-rotor, rotor], [-rotor, -rotor]]
  return (
    <group position={[p.x, y, p.z]} rotation={[0, (frame.heading_deg * Math.PI) / 180, 0]}>
      <mesh>
        <boxGeometry args={[1.1, 0.3, 1.1]} />
        <meshStandardMaterial color="#1f2937" />
      </mesh>
      {arms.map(([x, z], i) => (
        <group key={i} position={[x, 0.06, z]}>
          <mesh>
            <cylinderGeometry args={[0.05, 0.05, 0.12, 8]} />
            <meshStandardMaterial color="#374151" />
          </mesh>
          <mesh position={[0, 0.12, 0]}>
            <boxGeometry args={[1.1, 0.02, 0.09]} />
            <meshStandardMaterial color="#0ea5e9" transparent opacity={0.7} />
          </mesh>
        </group>
      ))}
      <pointLight color="#38bdf8" intensity={2} distance={4} />
    </group>
  )
}

function Route({ payload, xy, selected }: {
  payload: DemoPayload
  xy: (lat: number, lon: number) => { x: number; z: number }
  selected: number | null
}) {
  const { points, lineObject } = useMemo(() => {
    const pts = payload.mission.waypoints.map((w) => {
      const p = xy(w.lat_deg, w.lon_deg)
      return new THREE.Vector3(p.x, 0.25, p.z)
    })
    const geometry = new THREE.BufferGeometry().setFromPoints(pts)
    const material = new THREE.LineBasicMaterial({ color: '#f59e0b' })
    return { points: pts, lineObject: new THREE.Line(geometry, material) }
  }, [payload, xy])

  return (
    <group>
      <primitive object={lineObject} />
      {points.map((p, i) => (
        <group key={i} position={[p.x, 0.4, p.z]}>
          <mesh>
            <sphereGeometry args={[selected === i ? 0.8 : 0.45, 12, 12]} />
            <meshStandardMaterial color={selected === i ? '#ef4444' : '#fde047'} />
          </mesh>
          <Text position={[0, 1.1, 0]} fontSize={0.6} color="#ffffff">
            {i + 1}
          </Text>
        </group>
      ))}
    </group>
  )
}

function Detections({ payload, xy }: {
  payload: DemoPayload
  xy: (lat: number, lon: number) => { x: number; z: number }
}) {
  return (
    <group>
      {payload.detections.map((d) => {
        const p = xy(d.lat_deg, d.lon_deg)
        return (
          <mesh key={d.id} position={[p.x, 0.6, p.z]}>
            <octahedronGeometry args={[0.6]} />
            <meshStandardMaterial
              color={d.severity >= 0.2 ? '#dc2626' : '#84cc16'}
              emissive={d.severity >= 0.2 ? '#7f1d1d' : '#365314'}
            />
          </mesh>
        )
      })}
    </group>
  )
}

function Field({ payload, xy }: {
  payload: DemoPayload
  xy: (lat: number, lon: number) => { x: number; z: number }
}) {
  const wps = payload.mission.waypoints
  const lats = wps.map((w) => w.lat_deg)
  const lons = wps.map((w) => w.lon_deg)
  const minLat = Math.min(...lats, 36.8)
  const maxLat = Math.max(...lats, 36.8)
  const minLon = Math.min(...lons, 10.2)
  const maxLon = Math.max(...lons, 10.2)
  const margin = 1.15
  const a = xy(minLat - (maxLat - minLat) * (margin - 1), minLon - (maxLon - minLon) * (margin - 1))
  const b = xy(maxLat + (maxLat - minLat) * (margin - 1), maxLon + (maxLon - minLon) * (margin - 1))
  const width = Math.max(10, Math.abs(b.x - a.x))
  const depth = Math.max(10, Math.abs(b.z - a.z))
  return (
    <group>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
        <planeGeometry args={[width, depth]} />
        <meshStandardMaterial color="#3f6212" />
      </mesh>
      {/* crop rows */}
      {Array.from({ length: 12 }, (_, i) => (
        <mesh key={i} rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.02, -depth / 2 + (i + 0.5) * (depth / 12)]}>
          <planeGeometry args={[width * 0.95, 0.35]} />
          <meshStandardMaterial color="#65a30d" />
        </mesh>
      ))}
      {/* home marker */}
      <mesh position={[0, 0.3, 0]}>
        <cylinderGeometry args={[1.2, 1.2, 0.5, 16]} />
        <meshStandardMaterial color="#22d3ee" />
      </mesh>
      <Text position={[0, 1.4, 0]} fontSize={0.9} color="#22d3ee">HOME</Text>
    </group>
  )
}

export default function Scene({ payload, frame, selected, view }: {
  payload: DemoPayload
  frame: TelemetryFrame
  selected: number | null
  view: 'orbit' | 'top' | 'follow'
}) {
  const origin = useMemo(() => {
    const wps = payload.mission.waypoints
    if (!wps.length) return { lat_deg: 36.8, lon_deg: 10.2 }
    return {
      lat_deg: (Math.min(...wps.map((w) => w.lat_deg)) + Math.max(...wps.map((w) => w.lat_deg))) / 2,
      lon_deg: (Math.min(...wps.map((w) => w.lon_deg)) + Math.max(...wps.map((w) => w.lon_deg))) / 2,
    }
  }, [payload])
  const xy = useMemo(
    () => makeTransform(origin.lat_deg, origin.lon_deg),
    [origin.lat_deg, origin.lon_deg],
  )
  const dronePos = xy(frame.lat_deg, frame.lon_deg)
  const cameraPos: [number, number, number] =
    view === 'top' ? [0, 120, 0.01]
      : view === 'follow' ? [dronePos.x + 14, 18, dronePos.z + 18]
        : [0, 85, 55]
  const cameraTarget: [number, number, number] =
    view === 'follow' ? [dronePos.x, 2, dronePos.z] : [0, 0, 0]
  return (
    <Canvas shadows style={{ width: '100%', height: '100%' }} key={view}>
      <color attach="background" args={['#0b1120']} />
      <fog attach="fog" args={['#0b1120', 80, 220]} />
      <PerspectiveCamera makeDefault position={cameraPos} fov={50} />
      <OrbitControls makeDefault target={cameraTarget} enablePan enableZoom maxDistance={250} minDistance={8} />
      <ambientLight intensity={0.9} />
      <directionalLight position={[30, 60, 20]} intensity={1.6} castShadow />
      <Grid args={[400, 400]} cellColor="#1e293b" sectionColor="#334155" fadeDistance={160} />
      <Field payload={payload} xy={xy} />
      <Route payload={payload} xy={xy} selected={selected} />
      <Detections payload={payload} xy={xy} />
      <Drone frame={frame} xy={xy} />
    </Canvas>
  )
}
