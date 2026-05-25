import React, { useState, useEffect, useCallback, useMemo } from 'react'
import { useSimStore } from '../store'

// ── Individual checklist item definition ──
function useChecklist() {
  const drones = useSimStore(state => state.drones)
  const simState = useSimStore(state => state.simState)
  const isConnected = useSimStore(state => state.isConnected)
  const getDroneStats = useSimStore(state => state.getDroneStats)

  return useMemo(() => {
    const stats = getDroneStats()
    const items = [
      {
        id: 'connection',
        label: 'Backend Connection',
        detail: isConnected ? 'WebSocket open' : 'Disconnected',
        pass: isConnected
      },
      {
        id: 'drone_count',
        label: 'Drone Count > 0',
        detail: `${stats.total} drones registered`,
        pass: stats.total > 0
      },
      {
        id: 'rtk_fix',
        label: 'RTK GPS Fix (>80%)',
        detail: `${stats.rtkFixPct?.toFixed(0) ?? 0}% with fix`,
        pass: stats.total > 0 && (stats.rtkFixPct ?? 0) >= 80
      },
      {
        id: 'battery',
        label: 'Battery OK (>70%)',
        detail: `Avg: ${stats.avgBattery.toFixed(1)}%`,
        pass: stats.total > 0 && stats.avgBattery >= 70
      },
      {
        id: 'no_failsafe',
        label: 'No Failsafe Drones',
        detail: stats.failsafe > 0 ? `${stats.failsafe} in failsafe` : 'All nominal',
        pass: stats.total > 0 && stats.failsafe === 0
      },
      {
        id: 'show_loaded',
        label: 'Show File Loaded',
        detail: simState.showLoaded ? 'Show ready' : 'No show file',
        pass: !!simState.showLoaded
      },
      {
        id: 'geofence',
        label: 'Geofence Active',
        detail: 'Virtual boundary set',
        pass: true  // simulated - always active
      },
      {
        id: 'armed',
        label: 'All Drones Armed',
        detail: stats.total > 0
          ? `${stats.total - stats.idle - stats.grounded}/${stats.total} armed`
          : 'No drones',
        pass: stats.total > 0 && (stats.idle + stats.grounded) === 0
      }
    ]
    return items
  }, [drones, simState, isConnected, getDroneStats])
}

// ── HTTP helpers ──
async function apiPost(path, body = {}) {
  const res = await fetch(`/api${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  })
  if (!res.ok) {
    const text = await res.text()
    throw new Error(`API ${path} failed: ${res.status} ${text}`)
  }
  return res.json().catch(() => ({}))
}

async function apiPostForm(path, formData) {
  const res = await fetch(`/api${path}`, {
    method: 'POST',
    body: formData
  })
  if (!res.ok) {
    const text = await res.text()
    throw new Error(`API ${path} failed: ${res.status} ${text}`)
  }
  return res.json().catch(() => ({}))
}

// ── Toast notification (local) ──
function useToast() {
  const [toasts, setToasts] = useState([])

  const addToast = useCallback((message, type = 'info') => {
    const id = Date.now() + Math.random()
    setToasts(prev => [...prev, { id, message, type }])
    setTimeout(() => {
      setToasts(prev => prev.filter(t => t.id !== id))
    }, 3500)
  }, [])

  return { toasts, addToast }
}

export default function PreflightCheck({ onCommand }) {
  const simState = useSimStore(state => state.simState)
  const showInfo = useSimStore(state => state.showInfo)
  const isConnected = useSimStore(state => state.isConnected)
  const addLog = useSimStore(state => state.addLog)

  const checklist = useChecklist()
  const { toasts, addToast } = useToast()

  const [droneCount, setDroneCount] = useState(50)
  const [uploadFile, setUploadFile] = useState(null)
  const [loading, setLoading] = useState({})

  const passCount = checklist.filter(c => c.pass).length
  const allPass = passCount === checklist.length
  const scoreClass = allPass ? 'all-pass' : passCount >= 5 ? 'partial' : 'fail'

  const setLoadingKey = (key, val) =>
    setLoading(prev => ({ ...prev, [key]: val }))

  // ── Send WebSocket command ──
  const sendCommand = useCallback((action) => {
    if (onCommand) {
      onCommand({ type: 'CMD', action })
      addLog({
        _id: Date.now(),
        timestamp: Date.now() / 1000,
        level: 'INFO',
        subsystem: 'GCS',
        message: `Command sent: ${action}`
      })
    }
  }, [onCommand, addLog])

  // ── Apply drone count ──
  const handleApplyDroneCount = useCallback(async () => {
    setLoadingKey('droneCount', true)
    try {
      await apiPost('/set_drone_count', { count: droneCount })
      addToast(`Drone count set to ${droneCount}`, 'success')
      addLog({
        _id: Date.now(),
        timestamp: Date.now() / 1000,
        level: 'INFO',
        subsystem: 'GCS',
        message: `Set drone count to ${droneCount}`
      })
    } catch (e) {
      addToast(e.message, 'error')
    } finally {
      setLoadingKey('droneCount', false)
    }
  }, [droneCount, addToast, addLog])

  // ── Generate demo show ──
  const handleGenerateDemo = useCallback(async () => {
    setLoadingKey('demo', true)
    try {
      const result = await apiPost('/generate_demo')
      addToast('Demo show generated!', 'success')
      addLog({
        _id: Date.now(),
        timestamp: Date.now() / 1000,
        level: 'INFO',
        subsystem: 'GCS',
        message: `Demo show generated: ${result?.title ?? 'untitled'}`
      })
    } catch (e) {
      addToast(e.message, 'error')
    } finally {
      setLoadingKey('demo', false)
    }
  }, [addToast, addLog])

  // ── Upload show file ──
  const handleUpload = useCallback(async () => {
    if (!uploadFile) {
      addToast('No file selected.', 'error')
      return
    }
    setLoadingKey('upload', true)
    try {
      const fd = new FormData()
      fd.append('file', uploadFile)
      const result = await apiPostForm('/upload_show', fd)
      addToast(`Show uploaded: ${uploadFile.name}`, 'success')
      addLog({
        _id: Date.now(),
        timestamp: Date.now() / 1000,
        level: 'INFO',
        subsystem: 'GCS',
        message: `Show file uploaded: ${uploadFile.name}`
      })
      setUploadFile(null)
    } catch (e) {
      addToast(e.message, 'error')
    } finally {
      setLoadingKey('upload', false)
    }
  }, [uploadFile, addToast, addLog])

  return (
    <div className="preflight-container">
      {/* ── Checklist ── */}
      <div className="panel-section">
        <div className="panel-title" style={{ marginBottom: 8 }}>
          Preflight Checklist
        </div>
        <ul className="checklist">
          {checklist.map(item => (
            <li key={item.id} className="checklist-item">
              <span className={`check-icon ${item.pass ? 'pass' : 'fail'}`}>
                {item.pass ? '✓' : '✗'}
              </span>
              <span className="checklist-label">{item.label}</span>
              <span className="checklist-detail">{item.detail}</span>
            </li>
          ))}
        </ul>

        <div className="preflight-summary">
          <div>
            <div className="text-dim" style={{ fontSize: 10, textTransform: 'uppercase', letterSpacing: '0.06em', marginBottom: 2 }}>
              Preflight Score
            </div>
            <div className={`preflight-score ${scoreClass}`}>
              {passCount}/{checklist.length}
            </div>
          </div>
          <div style={{ textAlign: 'right' }}>
            <div className="text-dim" style={{ fontSize: 10, marginBottom: 2 }}>
              {allPass ? 'Ready to fly' : 'Not ready'}
            </div>
            <div style={{
              fontSize: 20,
              filter: allPass ? 'none' : 'grayscale(1) opacity(0.4)'
            }}>
              {allPass ? '✅' : '⚠️'}
            </div>
          </div>
        </div>
      </div>

      {/* ── Show info ── */}
      {showInfo && (
        <div className="panel-section">
          <div className="panel-title" style={{ marginBottom: 8 }}>Current Show</div>
          <div className="show-info-card">
            <div className="show-info-title">{showInfo.title || 'Untitled Show'}</div>
            <div className="show-info-meta">
              <div className="show-info-item">
                <span className="show-info-item-label">Duration</span>
                <span className="show-info-item-value">{formatDuration(showInfo.duration)}</span>
              </div>
              <div className="show-info-item">
                <span className="show-info-item-label">Drones</span>
                <span className="show-info-item-value">{showInfo.droneCount ?? '—'}</span>
              </div>
              <div className="show-info-item">
                <span className="show-info-item-label">Waypoints</span>
                <span className="show-info-item-value">{showInfo.waypointCount ?? '—'}</span>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* ── Drone count control ── */}
      <div className="panel-section">
        <div className="panel-title" style={{ marginBottom: 8 }}>Swarm Size</div>
        <div className="form-group">
          <div className="range-container">
            <input
              type="range"
              className="range-input"
              min={10}
              max={500}
              step={10}
              value={droneCount}
              onChange={e => setDroneCount(Number(e.target.value))}
            />
            <span className="range-value">{droneCount}</span>
          </div>
          <button
            className="btn btn-primary btn-sm"
            style={{ marginTop: 6, alignSelf: 'flex-start' }}
            onClick={handleApplyDroneCount}
            disabled={loading.droneCount || !isConnected}
          >
            {loading.droneCount ? <span className="loading-spinner" /> : null}
            Apply Count
          </button>
        </div>
      </div>

      {/* ── Show file upload ── */}
      <div className="panel-section">
        <div className="panel-title" style={{ marginBottom: 8 }}>Show File</div>
        <div className="upload-area" style={{ marginBottom: 8 }}>
          <input
            type="file"
            accept=".skyc,.csv,.json"
            onChange={e => setUploadFile(e.target.files[0] || null)}
          />
          <div className="upload-icon">📁</div>
          <div className="upload-text">
            {uploadFile ? uploadFile.name : 'Click or drag to upload'}
          </div>
          <div className="upload-hint">.skyc  .csv  .json</div>
        </div>
        <div className="btn-group">
          <button
            className="btn btn-primary btn-sm"
            onClick={handleUpload}
            disabled={!uploadFile || loading.upload}
          >
            {loading.upload ? <span className="loading-spinner" /> : null}
            Upload Show
          </button>
          <button
            className="btn btn-purple btn-sm"
            onClick={handleGenerateDemo}
            disabled={loading.demo || !isConnected}
          >
            {loading.demo ? <span className="loading-spinner" /> : null}
            Demo Show
          </button>
        </div>
      </div>

      {/* ── Show controls ── */}
      <div className="panel-section">
        <div className="panel-title" style={{ marginBottom: 8 }}>Show Control</div>
        <div className="btn-group">
          <button
            className="btn btn-warning btn-sm"
            onClick={() => sendCommand('arm_all')}
            disabled={!isConnected}
            title="Arm all drones"
          >
            ⚡ Arm All
          </button>
          <button
            className="btn btn-success btn-sm"
            onClick={() => sendCommand('start_show')}
            disabled={!isConnected || !allPass}
            title={allPass ? 'Start the show' : 'Complete preflight checklist first'}
          >
            ▶ Start Show
          </button>
        </div>
        <div className="btn-group" style={{ marginTop: 6 }}>
          <button
            className="btn btn-sm"
            onClick={() => sendCommand('stop_show')}
            disabled={!isConnected || !simState.running}
            title="Stop the show"
          >
            ■ Stop Show
          </button>
          <button
            className="btn btn-danger btn-sm"
            onClick={() => sendCommand('land_all')}
            disabled={!isConnected}
            title="Emergency land all drones"
          >
            ▼ Land All
          </button>
        </div>
      </div>

      {/* ── Local toasts ── */}
      <div style={{
        position: 'fixed',
        bottom: 16,
        left: 16,
        zIndex: 9999,
        display: 'flex',
        flexDirection: 'column',
        gap: 6,
        pointerEvents: 'none',
        maxWidth: 320
      }}>
        {toasts.map(t => (
          <div key={t.id} className={`toast ${t.type}`}>
            {t.message}
          </div>
        ))}
      </div>
    </div>
  )
}

function formatDuration(seconds) {
  if (!seconds) return '—'
  const m = Math.floor(seconds / 60)
  const s = Math.floor(seconds % 60)
  return `${m}m ${s}s`
}
