import React, { useMemo, useState, useCallback, useRef } from 'react'
import { useSimStore } from '../store'
import {
  BarChart, Bar, XAxis, YAxis, Tooltip, ResponsiveContainer, Cell
} from 'recharts'

// ── Status config ──
const STATUS_CONFIG = {
  FLYING:   { label: 'Flying',   color: '#34d399', bg: 'rgba(52,211,153,0.1)',  borderColor: 'rgba(52,211,153,0.3)' },
  IDLE:     { label: 'Idle',     color: '#94a3b8', bg: 'rgba(148,163,184,0.06)',borderColor: 'rgba(148,163,184,0.2)' },
  TAKEOFF:  { label: 'Takeoff',  color: '#60a5fa', bg: 'rgba(96,165,250,0.1)',  borderColor: 'rgba(96,165,250,0.3)' },
  LANDING:  { label: 'Landing',  color: '#fbbf24', bg: 'rgba(251,191,36,0.1)',  borderColor: 'rgba(251,191,36,0.3)' },
  FAILSAFE: { label: 'Failsafe', color: '#f87171', bg: 'rgba(248,113,113,0.12)',borderColor: 'rgba(248,113,113,0.4)' },
  GROUNDED: { label: 'Grounded', color: '#475569', bg: 'rgba(71,85,105,0.08)',  borderColor: 'rgba(71,85,105,0.2)' },
  ARMING:   { label: 'Arming',   color: '#22d3ee', bg: 'rgba(34,211,238,0.1)', borderColor: 'rgba(34,211,238,0.3)' },
  ERROR:    { label: 'Error',    color: '#e879f9', bg: 'rgba(232,121,249,0.1)', borderColor: 'rgba(232,121,249,0.3)' },
}

function getBatteryClass(pct) {
  if (pct >= 60) return 'high'
  if (pct >= 30) return 'medium'
  return 'low'
}

function rgbToHex(r, g, b) {
  return `rgb(${r ?? 255},${g ?? 255},${b ?? 255})`
}

// ── Battery distribution histogram ──
function buildBatteryHistogram(drones) {
  const buckets = Array.from({ length: 10 }, (_, i) => ({
    label: `${i * 10}-${i * 10 + 10}%`,
    range: `${i * 10}–${i * 10 + 10}`,
    count: 0,
    pct: i * 10
  }))
  for (const d of drones) {
    const b = Math.min(99.9, Math.max(0, d.battery ?? 100))
    const idx = Math.floor(b / 10)
    buckets[idx].count++
  }
  return buckets
}

function batteryBucketColor(pct) {
  if (pct >= 60) return '#10b981'
  if (pct >= 30) return '#f59e0b'
  return '#ef4444'
}

const CustomBarTooltip = ({ active, payload }) => {
  if (!active || !payload?.length) return null
  const { range, count } = payload[0].payload
  return (
    <div style={{
      background: 'var(--bg-panel)',
      border: '1px solid var(--border-bright)',
      borderRadius: 4,
      padding: '6px 10px',
      fontSize: 11,
      fontFamily: 'var(--font-mono)',
      color: 'var(--text)',
      boxShadow: '0 4px 16px rgba(0,0,0,0.4)'
    }}>
      <div style={{ color: 'var(--text-dim)', marginBottom: 2 }}>{range}</div>
      <div><b>{count}</b> drones</div>
    </div>
  )
}

// ── Drone pixel tooltip ──
function DroneTooltip({ drone, x, y }) {
  if (!drone) return null
  const statusConf = STATUS_CONFIG[drone.status] || STATUS_CONFIG.IDLE
  return (
    <div
      className="drone-tooltip"
      style={{ left: x + 12, top: y - 10 }}
    >
      <div className="drone-tooltip-title">Drone #{drone.id}</div>
      <div className="drone-tooltip-row">
        <span>Status</span>
        <span style={{ color: statusConf.color }}>{drone.status || 'UNKNOWN'}</span>
      </div>
      <div className="drone-tooltip-row">
        <span>Battery</span>
        <span style={{ color: getBatteryClass(drone.battery) === 'low' ? '#f87171' : getBatteryClass(drone.battery) === 'medium' ? '#fbbf24' : '#34d399' }}>
          {(drone.battery ?? 0).toFixed(1)}%
        </span>
      </div>
      <div className="drone-tooltip-row">
        <span>X / Y / Z</span>
        <span>{(drone.x ?? 0).toFixed(1)} / {(drone.y ?? 0).toFixed(1)} / {(drone.z ?? 0).toFixed(1)}</span>
      </div>
      <div className="drone-tooltip-row">
        <span>RTK Fix</span>
        <span style={{ color: drone.rtk_fix ? '#34d399' : '#f87171' }}>
          {drone.rtk_fix ? 'YES' : 'NO'}
        </span>
      </div>
      <div className="drone-tooltip-row">
        <span>LED</span>
        <span>
          <span style={{
            display: 'inline-block',
            width: 8,
            height: 8,
            borderRadius: 2,
            background: rgbToHex(drone.r, drone.g, drone.b),
            verticalAlign: 'middle',
            marginRight: 4,
            border: '1px solid rgba(255,255,255,0.2)'
          }} />
          rgb({drone.r ?? 0},{drone.g ?? 0},{drone.b ?? 0})
        </span>
      </div>
    </div>
  )
}

// ── Sort icon ──
function SortIcon({ direction }) {
  if (!direction) return <span style={{ opacity: 0.3, fontSize: 9 }}>↕</span>
  return <span style={{ fontSize: 9 }}>{direction === 'asc' ? '↑' : '↓'}</span>
}

export default function SwarmGrid() {
  const drones = useSimStore(state => state.drones)
  const getDroneStats = useSimStore(state => state.getDroneStats)

  const [tooltip, setTooltip] = useState(null) // { drone, x, y }
  const [sortKey, setSortKey] = useState('battery')
  const [sortDir, setSortDir] = useState('asc')
  const tooltipTimeout = useRef(null)

  const stats = useMemo(() => getDroneStats(), [drones, getDroneStats])
  const batteryHisto = useMemo(() => buildBatteryHistogram(drones), [drones])

  // Sorted table data (worst 20 by battery)
  const tableRows = useMemo(() => {
    const sorted = [...drones].sort((a, b) => {
      let va = a[sortKey] ?? 0
      let vb = b[sortKey] ?? 0
      if (typeof va === 'boolean') va = va ? 1 : 0
      if (typeof vb === 'boolean') vb = vb ? 1 : 0
      return sortDir === 'asc' ? va - vb : vb - va
    })
    return sorted.slice(0, 20)
  }, [drones, sortKey, sortDir])

  const handleSort = useCallback((key) => {
    setSortKey(prev => {
      if (prev === key) {
        setSortDir(d => d === 'asc' ? 'desc' : 'asc')
        return key
      }
      setSortDir('asc')
      return key
    })
  }, [])

  const handlePixelEnter = useCallback((drone, e) => {
    if (tooltipTimeout.current) clearTimeout(tooltipTimeout.current)
    setTooltip({ drone, x: e.clientX, y: e.clientY })
  }, [])

  const handlePixelMove = useCallback((e) => {
    setTooltip(prev => prev ? { ...prev, x: e.clientX, y: e.clientY } : null)
  }, [])

  const handlePixelLeave = useCallback(() => {
    tooltipTimeout.current = setTimeout(() => setTooltip(null), 100)
  }, [])

  // Status summary counts
  const statusCounts = useMemo(() => {
    const counts = {}
    for (const d of drones) {
      counts[d.status] = (counts[d.status] || 0) + 1
    }
    return counts
  }, [drones])

  if (drones.length === 0) {
    return (
      <div className="swarm-container">
        <div style={{
          padding: '40px 20px',
          textAlign: 'center',
          color: 'var(--text-muted)',
          fontSize: 13
        }}>
          No drones connected.
          <br />
          <span style={{ fontSize: 11, color: 'var(--text-muted)', marginTop: 8, display: 'block' }}>
            Start the simulator or connect to backend to see swarm data.
          </span>
        </div>
      </div>
    )
  }

  return (
    <div className="swarm-container">
      {/* ── Status summary bar ── */}
      <div className="swarm-summary-bar">
        {Object.entries(statusCounts).map(([status, count]) => {
          const conf = STATUS_CONFIG[status]
          const cls = status.toLowerCase()
          return (
            <div key={status} className={`summary-chip ${cls}`}>
              <span>{count}</span>
              <span style={{ fontWeight: 400, opacity: 0.8, fontSize: 10 }}>
                {conf ? conf.label : status}
              </span>
            </div>
          )
        })}
      </div>

      {/* ── Statistics row ── */}
      <div className="stats-row">
        <div className="stat-cell">
          <span className="stat-cell-value accent">{stats.total}</span>
          <span className="stat-cell-label">Total</span>
        </div>
        <div className="stat-cell">
          <span className={`stat-cell-value ${stats.avgBattery > 60 ? 'success' : stats.avgBattery > 30 ? 'warning' : 'danger'}`}>
            {stats.avgBattery.toFixed(0)}%
          </span>
          <span className="stat-cell-label">Avg Batt</span>
        </div>
        <div className="stat-cell">
          <span className={`stat-cell-value ${stats.minBattery > 30 ? 'warning' : 'danger'}`}>
            {stats.minBattery.toFixed(0)}%
          </span>
          <span className="stat-cell-label">Min Batt</span>
        </div>
        <div className="stat-cell">
          <span className="stat-cell-value success">{stats.flying}</span>
          <span className="stat-cell-label">Flying</span>
        </div>
        <div className="stat-cell">
          <span className={`stat-cell-value ${stats.failsafe > 0 ? 'danger' : 'success'}`}>
            {stats.failsafe}
          </span>
          <span className="stat-cell-label">Failsafe</span>
        </div>
      </div>

      {/* ── Battery distribution chart ── */}
      <div className="chart-section">
        <div className="chart-section-title">Battery Distribution</div>
        <ResponsiveContainer width="100%" height={80}>
          <BarChart data={batteryHisto} margin={{ top: 2, right: 4, left: -30, bottom: 0 }}>
            <XAxis
              dataKey="label"
              tick={{ fontSize: 8, fill: '#4b5563' }}
              interval={1}
              tickLine={false}
              axisLine={false}
            />
            <YAxis
              tick={{ fontSize: 8, fill: '#4b5563' }}
              tickLine={false}
              axisLine={false}
              allowDecimals={false}
            />
            <Tooltip content={<CustomBarTooltip />} cursor={{ fill: 'rgba(255,255,255,0.04)' }} />
            <Bar dataKey="count" radius={[2, 2, 0, 0]}>
              {batteryHisto.map((entry) => (
                <Cell key={entry.label} fill={batteryBucketColor(entry.pct)} />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </div>

      {/* ── Mini drone pixel grid ── */}
      <div className="drone-mini-grid-container">
        <div className="chart-section-title" style={{ marginBottom: 8 }}>
          Swarm Map ({drones.length} drones)
          <span style={{ color: 'var(--text-muted)', marginLeft: 6, fontWeight: 400, fontSize: 9 }}>
            hover for details
          </span>
        </div>
        <div className="drone-mini-grid">
          {drones.map(drone => {
            const color = rgbToHex(drone.r, drone.g, drone.b)
            const isFailsafe = drone.status === 'FAILSAFE'
            return (
              <div
                key={drone.id}
                className={`drone-pixel ${isFailsafe ? 'failsafe' : ''}`}
                style={{
                  background: color,
                  opacity: drone.status === 'IDLE' || drone.status === 'GROUNDED' ? 0.3 : 1,
                  outline: isFailsafe ? '1px solid #f87171' : 'none'
                }}
                onMouseEnter={e => handlePixelEnter(drone, e)}
                onMouseMove={handlePixelMove}
                onMouseLeave={handlePixelLeave}
              />
            )
          })}
        </div>
      </div>

      {/* ── Sortable table - bottom 20 by battery ── */}
      <div className="drone-table-section">
        <div className="chart-section-title" style={{ marginBottom: 6 }}>
          {sortKey === 'battery' && sortDir === 'asc' ? 'Lowest Battery' : 'Drone Details'}
          <span style={{ color: 'var(--text-muted)', marginLeft: 6, fontWeight: 400, fontSize: 9 }}>
            top 20 shown
          </span>
        </div>
        <table className="drone-table">
          <thead>
            <tr>
              <th
                className={sortKey === 'id' ? 'sorted' : ''}
                onClick={() => handleSort('id')}
              >
                ID <SortIcon direction={sortKey === 'id' ? sortDir : null} />
              </th>
              <th>Status</th>
              <th
                className={sortKey === 'battery' ? 'sorted' : ''}
                onClick={() => handleSort('battery')}
              >
                Battery <SortIcon direction={sortKey === 'battery' ? sortDir : null} />
              </th>
              <th
                className={sortKey === 'z' ? 'sorted' : ''}
                onClick={() => handleSort('z')}
              >
                Alt <SortIcon direction={sortKey === 'z' ? sortDir : null} />
              </th>
              <th>X / Y</th>
              <th>RTK</th>
            </tr>
          </thead>
          <tbody>
            {tableRows.map(drone => {
              const battClass = getBatteryClass(drone.battery ?? 100)
              const statusConf = STATUS_CONFIG[drone.status] || STATUS_CONFIG.IDLE
              return (
                <tr key={drone.id}>
                  <td style={{ fontFamily: 'var(--font-mono)', color: 'var(--text-dim)' }}>
                    {drone.id}
                  </td>
                  <td>
                    <span
                      className="status-badge"
                      style={{
                        color: statusConf.color,
                        borderColor: statusConf.borderColor,
                        background: statusConf.bg
                      }}
                    >
                      {drone.status || 'UNKNOWN'}
                    </span>
                  </td>
                  <td>
                    <div className="battery-bar-container">
                      <div className="battery-bar">
                        <div
                          className={`battery-fill ${battClass}`}
                          style={{ width: `${Math.max(0, Math.min(100, drone.battery ?? 0))}%` }}
                        />
                      </div>
                      <span className="battery-text">{(drone.battery ?? 0).toFixed(0)}%</span>
                    </div>
                  </td>
                  <td style={{ fontFamily: 'var(--font-mono)', fontSize: 10 }}>
                    {(drone.z ?? 0).toFixed(1)}m
                  </td>
                  <td style={{ fontFamily: 'var(--font-mono)', fontSize: 10, color: 'var(--text-dim)' }}>
                    {(drone.x ?? 0).toFixed(1)}/{(drone.y ?? 0).toFixed(1)}
                  </td>
                  <td className="rtk-icon" style={{ color: drone.rtk_fix ? '#34d399' : '#f87171' }}>
                    {drone.rtk_fix ? '✓' : '✗'}
                  </td>
                </tr>
              )
            })}
          </tbody>
        </table>
      </div>

      {/* Tooltip */}
      {tooltip && (
        <DroneTooltip drone={tooltip.drone} x={tooltip.x} y={tooltip.y} />
      )}
    </div>
  )
}
