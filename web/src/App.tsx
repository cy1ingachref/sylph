import { useEffect, useMemo, useState } from 'react'
import Scene from './Scene'
import { validatePayload, type DemoPayload, type TelemetryFrame } from './types'

const SCENARIOS = ['normal', 'low_battery', 'wind_gust'] as const
type ScenarioName = (typeof SCENARIOS)[number]

async function loadScenario(name: ScenarioName): Promise<DemoPayload> {
  const res = await fetch(`/scenarios/${name}.json`)
  if (!res.ok) throw new Error(`failed to load ${name}: HTTP ${res.status}`)
  return validatePayload(await res.json())
}

function TelemetryCard({ label, value, unit, status }: {
  label: string; value: string; unit: string; status?: 'ok' | 'warn' | 'bad'
}) {
  return (
    <div className={`card ${status ?? 'ok'}`} data-testid={`card-${label}`}>
      <div className="card-label">{label}</div>
      <div className="card-value"><span data-testid={`value-${label}`}>{value}</span><small>{unit}</small></div>
    </div>
  )
}

export default function App() {
  const [scenario, setScenario] = useState<ScenarioName>('normal')
  const [payload, setPayload] = useState<DemoPayload | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [index, setIndex] = useState(0)
  const [playing, setPlaying] = useState(false)
  const [speed, setSpeed] = useState(1)
  const [selected, setSelected] = useState<number | null>(null)
  const [view, setView] = useState<'orbit' | 'top' | 'follow'>('orbit')

  useEffect(() => {
    let active = true
    loadScenario(scenario)
      .then((p) => { if (active) { setPayload(p); setIndex(0); setError(null) } })
      .catch((e) => { if (active) setError(String(e)) })
    return () => { active = false }
  }, [scenario])

  useEffect(() => {
    if (!playing || !payload) return
    const timer = window.setInterval(() => {
      setIndex((i) => {
        if (i >= payload.frames.length - 1) { setPlaying(false); return i }
        return i + 1
      })
    }, 500 / speed)
    return () => window.clearInterval(timer)
  }, [playing, speed, payload])

  const frames = payload?.frames ?? []
  const frame: TelemetryFrame | null = frames[Math.min(index, frames.length - 1)] ?? null
  const frameIndex = Math.min(index, Math.max(0, frames.length - 1))
  const events = useMemo(
    () => (payload?.events ?? []).filter((e) => e.t_s <= (frame?.t_s ?? -1)),
    [payload, frame],
  )

  if (error) return <div className="error" role="alert">Demo data error: {error}</div>
  if (!payload || !frame) return <div className="loading">Loading simulation…</div>

  const batteryStatus = frame.battery_pct < 20 ? 'bad' : frame.battery_pct < 40 ? 'warn' : 'ok'
  const lastEvent = events[events.length - 1]

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand">SYLPH <span>GROUND STATION</span></div>
        <div className="sim-badge" data-testid="simulation-badge">SIMULATION — NO REAL FLIGHT CONTROL</div>
        <div className="top-status">
          <label htmlFor="scenario-select">Scenario</label>
          <select
            id="scenario-select"
            data-testid="scenario-select"
            value={scenario}
            onChange={(e) => setScenario(e.target.value as ScenarioName)}
          >
            {SCENARIOS.map((s) => <option key={s} value={s}>{s}</option>)}
          </select>
          <span className="pill">mode: {payload.mode}</span>
          <span className="pill">schema v{payload.schema_version}</span>
        </div>
      </header>

      <main className="layout">
        <aside className="panel left" data-testid="left-panel">
          <h2>Mission</h2>
          <div className="kv"><span>ID</span><strong>{payload.mission_id}</strong></div>
          <div className="kv"><span>State</span><strong>{payload.mission.state}</strong></div>
          <div className="kv"><span>Waypoints</span><strong>{payload.mission.waypoints.length}</strong></div>
          <h3>Waypoint list</h3>
          <ul className="waypoints" data-testid="waypoint-list">
            {payload.mission.waypoints.map((w, i) => (
              <li key={w.id}>
                <button
                  type="button"
                  data-testid={`waypoint-${i}`}
                  className={selected === i ? 'selected' : ''}
                  onClick={() => setSelected(i === selected ? null : i)}
                >
                  {i + 1}. {w.id} — {w.lat_deg.toFixed(5)}, {w.lon_deg.toFixed(5)}
                </button>
              </li>
            ))}
          </ul>
        </aside>

        <section className="scene" data-testid="scene">
          <div className="view-buttons" data-testid="view-buttons">
            {(['orbit', 'top', 'follow'] as const).map((v) => (
              <button
                key={v}
                type="button"
                data-testid={`view-${v}`}
                className={view === v ? 'active' : ''}
                onClick={() => setView(v)}
              >
                {v}
              </button>
            ))}
          </div>
          <Scene payload={payload} frame={frame} selected={selected} view={view} />
          <div className="scene-caption">
            {selected !== null
              ? `Selected ${payload.mission.waypoints[selected]?.id ?? selected}`
              : 'Orbit: drag · Pan: right-drag · Zoom: wheel'}
          </div>
        </section>

        <aside className="panel right" data-testid="right-panel">
          <h2>Vehicle</h2>
          <div className="cards">
            <TelemetryCard label="Battery" value={frame.battery_pct.toFixed(1)} unit="%" status={batteryStatus} />
            <TelemetryCard label="Altitude" value={frame.alt_m.toFixed(1)} unit="m" />
            <TelemetryCard label="Speed" value={frame.ground_speed_mps.toFixed(1)} unit="m/s" />
            <TelemetryCard label="Heading" value={frame.heading_deg.toFixed(0)} unit="deg" />
            <TelemetryCard label="BatteryWh" value={frame.battery_wh.toFixed(1)} unit="Wh" status={batteryStatus} />
            <TelemetryCard label="Waypoint" value={String(frame.current_wp)} unit="#" />
          </div>
          <h3>Live events</h3>
          <ul className="events" data-testid="event-list">
            {events.slice(-6).map((e, i) => (
              <li key={i}><span className={`dot ${e.kind}`} />{e.kind}: {e.detail}</li>
            ))}
          </ul>
          <div className="last-event" data-testid="last-event">{lastEvent?.kind ?? 'none'}</div>
        </aside>
      </main>

      <footer className="timeline" data-testid="timeline">
        <button type="button" data-testid="play" onClick={() => setPlaying((p) => !p)}>
          {playing ? 'Pause' : 'Play'}
        </button>
        <button type="button" data-testid="reset" onClick={() => { setPlaying(false); setIndex(0) }}>Reset</button>
        <label htmlFor="speed">Speed</label>
        <select id="speed" data-testid="speed" value={speed} onChange={(e) => setSpeed(Number(e.target.value))}>
          {[0.5, 1, 2, 4].map((s) => <option key={s} value={s}>{s}×</option>)}
        </select>
        <input
          type="range" min={0} max={Math.max(0, frames.length - 1)} value={frameIndex}
          data-testid="scrub"
          onChange={(e) => { setPlaying(false); setIndex(Number(e.target.value)) }}
        />
        <span className="clock">{frame.t_s.toFixed(1)} s · frame {frameIndex + 1}/{frames.length}</span>
      </footer>
    </div>
  )
}
