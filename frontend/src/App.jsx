import React, { useEffect, useRef, useState, useCallback } from 'react'
import PreflightCheck from './dashboard/PreflightCheck'
import SwarmGrid from './dashboard/SwarmGrid'
import LogViewer from './dashboard/LogViewer'
import { SimulatorCanvas } from './visualizer/SceneManager.jsx'
import { WebSocketClient } from './visualizer/WebSocketClient'
import { useSimStore } from './store'

// ── Connection status indicator ──
function ConnectionStatus({ status }) {
  const label = status === 'connected' ? 'CONNECTED'
    : status === 'connecting' ? 'CONNECTING'
    : 'DISCONNECTED'

  return (
    <div className={`connection-indicator ${status}`}>
      <span className="conn-dot" />
      {label}
    </div>
  )
}

// ── Tab badge ──
function TabBadge({ count }) {
  if (!count) return null
  return <span className="tab-badge">{count > 99 ? '99+' : count}</span>
}

export default function App() {
  const [activeTab, setActiveTab] = useState('preflight')
  const [connStatus, setConnStatus] = useState('disconnected')

  const wsClientRef = useRef(null)

  const setDrones = useSimStore(state => state.setDrones)
  const setSimState = useSimStore(state => state.setSimState)
  const setConnected = useSimStore(state => state.setConnected)
  const addLog = useSimStore(state => state.addLog)
  const setShowInfo = useSimStore(state => state.setShowInfo)

  // Derived stats for header
  const drones = useSimStore(state => state.drones)
  const simState = useSimStore(state => state.simState)
  const logs = useSimStore(state => state.logs)

  const droneCount = drones.length
  const avgBattery = droneCount > 0
    ? (drones.reduce((s, d) => s + (d.battery ?? 100), 0) / droneCount).toFixed(1)
    : '—'
  const failsafeCount = drones.filter(d => d.status === 'FAILSAFE').length
  const errorLogCount = logs.filter(l => l.level === 'ERROR' || l.level === 'CRITICAL').length

  // Format show time
  const showTime = simState.showTime || 0
  const showTimeStr = formatTime(showTime)

  // ── WebSocket message handler ──
  const handleMessage = useCallback((data) => {
    if (!data || !data.type) return

    switch (data.type) {
      case 'TEL':
        // Main telemetry update
        if (Array.isArray(data.drones)) {
          setDrones(data.drones)
        }
        break

      case 'STATE':
        // Simulator state update
        if (data.state) {
          setSimState(data.state)
        }
        break

      case 'LOG':
        // Log entry from backend
        addLog({
          _id: (data.timestamp ?? Date.now()) + Math.random(),
          timestamp: data.timestamp,
          level: data.level || 'INFO',
          subsystem: data.subsystem || 'SIM',
          drone_id: data.drone_id,
          message: data.message || ''
        })
        break

      case 'SHOW_INFO':
        if (data.info) {
          setShowInfo(data.info)
          setSimState({ showLoaded: true })
        }
        break

      case 'PONG':
        // Heartbeat response - ignore
        break

      default:
        // Unknown message type - log it
        console.debug('[WS] Unknown message type:', data.type, data)
        break
    }
  }, [setDrones, setSimState, addLog, setShowInfo])

  // ── WebSocket connection ──
  useEffect(() => {
    const wsUrl = `ws://${window.location.hostname}:8000/ws`

    setConnStatus('connecting')

    const client = new WebSocketClient(
      wsUrl,
      handleMessage,
      () => {
        // onConnect
        setConnected(true)
        setConnStatus('connected')
        addLog({
          _id: Date.now(),
          timestamp: Date.now() / 1000,
          level: 'INFO',
          subsystem: 'GCS',
          message: `WebSocket connected to ${wsUrl}`
        })
      },
      () => {
        // onDisconnect
        setConnected(false)
        setConnStatus('disconnected')
        addLog({
          _id: Date.now(),
          timestamp: Date.now() / 1000,
          level: 'WARNING',
          subsystem: 'GCS',
          message: 'WebSocket disconnected. Attempting to reconnect...'
        })
      }
    )

    client.connect()
    wsClientRef.current = client

    // Poll /api/status for sim state periodically
    let statusPollTimer = null
    const pollStatus = async () => {
      try {
        const res = await fetch('/api/status', { signal: AbortSignal.timeout(3000) })
        if (res.ok) {
          const data = await res.json()
          if (data) {
            setSimState(data)
            if (data.showInfo) setShowInfo(data.showInfo)
          }
        }
      } catch (e) {
        // silently ignore - WS gives us most info
      }
      statusPollTimer = setTimeout(pollStatus, 5000)
    }
    pollStatus()

    // Fetch SSE logs stream
    let eventSource = null
    try {
      eventSource = new EventSource('/api/logs/stream')
      eventSource.onmessage = (e) => {
        try {
          const entry = JSON.parse(e.data)
          addLog({
            _id: (entry.timestamp ?? Date.now()) + Math.random(),
            timestamp: entry.timestamp,
            level: entry.level || 'INFO',
            subsystem: entry.subsystem || 'SIM',
            drone_id: entry.drone_id,
            message: entry.message || ''
          })
        } catch (err) {
          // ignore parse errors
        }
      }
      eventSource.onerror = () => {
        // SSE errors are normal if backend is down - don't spam
      }
    } catch (e) {
      // EventSource not available or CORS issue - ignore
    }

    return () => {
      client.disconnect()
      wsClientRef.current = null
      if (statusPollTimer) clearTimeout(statusPollTimer)
      if (eventSource) eventSource.close()
    }
  }, [handleMessage, setConnected, setSimState, addLog, setShowInfo])

  // ── Send command via WebSocket ──
  const handleCommand = useCallback((cmd) => {
    if (wsClientRef.current) {
      wsClientRef.current.send(cmd)
    }
  }, [])

  return (
    <div className="app-layout">
      {/* ── Header bar ── */}
      <header className="app-header">
        <div className="app-header-logo">
          <span className="logo-icon">◈</span>
          Drone GCS
        </div>
        <div className="app-header-divider" />
        <div className="app-header-stats">
          <div className="header-stat">
            <span className="header-stat-label">Drones</span>
            <span className="header-stat-value accent">{droneCount}</span>
          </div>
          <div className="app-header-divider" />
          <div className="header-stat">
            <span className="header-stat-label">Show Time</span>
            <span className={`header-stat-value ${simState.running ? 'accent' : ''}`}>
              {showTimeStr}
            </span>
          </div>
          <div className="app-header-divider" />
          <div className="header-stat">
            <span className="header-stat-label">Avg Battery</span>
            <span className={`header-stat-value ${
              avgBattery === '—' ? '' :
              Number(avgBattery) > 60 ? 'success' :
              Number(avgBattery) > 30 ? 'warning' : 'danger'
            }`}>
              {avgBattery}{avgBattery !== '—' ? '%' : ''}
            </span>
          </div>
          {failsafeCount > 0 && (
            <>
              <div className="app-header-divider" />
              <div className="header-stat">
                <span className="header-stat-label">Failsafe</span>
                <span className="header-stat-value danger">{failsafeCount}</span>
              </div>
            </>
          )}
          {simState.running && (
            <>
              <div className="app-header-divider" />
              <div className="header-stat">
                <span className="header-stat-label">RTF</span>
                <span className="header-stat-value">{(simState.realTimeFactor || 1).toFixed(2)}x</span>
              </div>
            </>
          )}
        </div>

        <div className="app-header-right">
          {simState.running && (
            <div style={{
              background: 'rgba(16, 185, 129, 0.12)',
              border: '1px solid var(--success)',
              color: 'var(--success)',
              borderRadius: 'var(--radius)',
              padding: '3px 9px',
              fontSize: 11,
              fontWeight: 700,
              letterSpacing: '0.06em',
              animation: 'pulse-dot 2s infinite'
            }}>
              ● LIVE
            </div>
          )}
          <ConnectionStatus status={connStatus} />
        </div>
      </header>

      {/* ── Left sidebar ── */}
      <aside className="left-panel">
        <nav className="tabs">
          <button
            className={`tab-btn ${activeTab === 'preflight' ? 'active' : ''}`}
            onClick={() => setActiveTab('preflight')}
          >
            Preflight
          </button>
          <button
            className={`tab-btn ${activeTab === 'swarm' ? 'active' : ''}`}
            onClick={() => setActiveTab('swarm')}
          >
            Swarm
            {failsafeCount > 0 && <TabBadge count={failsafeCount} />}
          </button>
          <button
            className={`tab-btn ${activeTab === 'logs' ? 'active' : ''}`}
            onClick={() => setActiveTab('logs')}
          >
            Logs
            {errorLogCount > 0 && <TabBadge count={errorLogCount} />}
          </button>
        </nav>

        <div className="tab-content">
          {activeTab === 'preflight' && (
            <PreflightCheck onCommand={handleCommand} />
          )}
          {activeTab === 'swarm' && (
            <SwarmGrid />
          )}
          {activeTab === 'logs' && (
            <LogViewer />
          )}
        </div>
      </aside>

      {/* ── 3D Canvas ── */}
      <main className="canvas-area">
        <SimulatorCanvas />
      </main>
    </div>
  )
}

function formatTime(seconds) {
  if (!seconds || isNaN(seconds)) return '00:00'
  const m = Math.floor(seconds / 60)
  const s = Math.floor(seconds % 60)
  return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`
}
