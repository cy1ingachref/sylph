// Versioned contract from `python -m sylph.cli scenario` (schema_version=1).
// Units are explicit in field names. The renderer never invents movement:
// every drone position comes from a TelemetryFrame produced by the simulator.

export interface Position {
  lat_deg: number
  lon_deg: number
  alt_m: number
}

export interface TelemetryFrame {
  t_s: number
  lat_deg: number
  lon_deg: number
  alt_m: number
  ground_speed_mps: number
  heading_deg: number
  battery_pct: number
  battery_wh: number
  current_wp: number
  mode: string
}

export interface MissionEvent {
  t_s: number
  kind: string
  detail: string
  lat_deg: number | null
  lon_deg: number | null
}

export interface Waypoint {
  id: string
  lat_deg: number
  lon_deg: number
  alt_m: number
}

export interface Detection {
  id: string
  label: string
  confidence: number
  severity: number
  lat_deg: number
  lon_deg: number
  t_s: number
}

export interface DemoPayload {
  schema_version: number
  mode: string
  mission_id: string
  simulation_notice: string
  vehicle: {
    position: Position
    heading_deg: number
    speed_mps: number
    battery_pct: number
    battery_wh: number
    state: string
  }
  mission: {
    state: string
    current_waypoint: number
    waypoints: Waypoint[]
  }
  events: MissionEvent[]
  detections: Detection[]
  frames: TelemetryFrame[]
}

export function validatePayload(value: unknown): DemoPayload {
  if (typeof value !== 'object' || value === null) throw new Error('payload is not an object')
  const p = value as Partial<DemoPayload>
  if (p.schema_version !== 1) throw new Error(`unsupported schema_version: ${p.schema_version}`)
  if (!Array.isArray(p.frames) || p.frames.length === 0) throw new Error('no telemetry frames')
  if (!Array.isArray(p.events)) throw new Error('events missing')
  if (!Array.isArray(p.detections)) throw new Error('detections missing')
  const wps = p.mission?.waypoints
  if (!Array.isArray(wps)) throw new Error('waypoints missing')
  for (const f of p.frames) {
    if (!Number.isFinite(f.t_s) || !Number.isFinite(f.lat_deg) || !Number.isFinite(f.lon_deg)) {
      throw new Error('frame has non-finite position/time — refusing to invent values')
    }
  }
  return p as DemoPayload
}
