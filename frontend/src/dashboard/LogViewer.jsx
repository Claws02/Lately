import React, { useEffect, useRef, useState, useMemo, useCallback } from 'react'
import { useSimStore } from '../store'

const LEVELS = ['ALL', 'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']

const LEVEL_ORDER = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3, CRITICAL: 4 }

function formatTimestamp(ts) {
  if (!ts) return '--:--:--.---'
  const d = new Date(ts * 1000)
  const hh = String(d.getHours()).padStart(2, '0')
  const mm = String(d.getMinutes()).padStart(2, '0')
  const ss = String(d.getSeconds()).padStart(2, '0')
  const ms = String(d.getMilliseconds()).padStart(3, '0')
  return `${hh}:${mm}:${ss}.${ms}`
}

export default function LogViewer() {
  const logs = useSimStore(state => state.logs)
  const clearLogs = useSimStore(state => state.clearLogs)

  const [selectedLevel, setSelectedLevel] = useState('ALL')
  const [selectedSubsystem, setSelectedSubsystem] = useState('ALL')
  const [searchText, setSearchText] = useState('')
  const [autoScroll, setAutoScroll] = useState(true)

  const containerRef = useRef(null)
  const bottomRef = useRef(null)
  const userScrolled = useRef(false)

  // Gather unique subsystems
  const subsystems = useMemo(() => {
    const set = new Set()
    for (const log of logs) {
      if (log.subsystem) set.add(log.subsystem)
    }
    return ['ALL', ...Array.from(set).sort()]
  }, [logs])

  // Filter logs
  const filteredLogs = useMemo(() => {
    const minLevel = selectedLevel === 'ALL' ? -1 : (LEVEL_ORDER[selectedLevel] ?? -1)
    const lowerSearch = searchText.toLowerCase()
    return logs.filter(entry => {
      if (selectedLevel !== 'ALL') {
        const entryLevel = LEVEL_ORDER[entry.level] ?? 0
        if (entryLevel < minLevel) return false
      }
      if (selectedSubsystem !== 'ALL' && entry.subsystem !== selectedSubsystem) return false
      if (searchText && !entry.message?.toLowerCase().includes(lowerSearch)) return false
      return true
    })
  }, [logs, selectedLevel, selectedSubsystem, searchText])

  // Count errors for badge
  const errorCount = useMemo(() =>
    logs.filter(l => l.level === 'ERROR' || l.level === 'CRITICAL').length,
    [logs]
  )

  // Auto-scroll logic
  useEffect(() => {
    if (!autoScroll) return
    if (bottomRef.current) {
      bottomRef.current.scrollIntoView({ behavior: 'instant', block: 'end' })
    }
  }, [filteredLogs, autoScroll])

  const handleScroll = useCallback(() => {
    const container = containerRef.current
    if (!container) return
    const nearBottom = container.scrollHeight - container.scrollTop - container.clientHeight < 40
    setAutoScroll(nearBottom)
  }, [])

  const handleClear = useCallback(() => {
    clearLogs()
  }, [clearLogs])

  return (
    <div className="log-viewer-container">
      {/* Toolbar */}
      <div className="log-toolbar">
        {/* Level filters */}
        <div className="log-filters">
          {LEVELS.map(level => (
            <button
              key={level}
              className={`log-level-btn ${selectedLevel === level ? `active ${level}` : ''}`}
              onClick={() => setSelectedLevel(level)}
            >
              {level}
            </button>
          ))}
          {errorCount > 0 && (
            <span style={{
              marginLeft: 'auto',
              background: 'rgba(239,68,68,0.2)',
              color: '#f87171',
              border: '1px solid rgba(248,113,113,0.4)',
              borderRadius: '3px',
              padding: '2px 6px',
              fontSize: '10px',
              fontWeight: 700,
              fontFamily: 'var(--font-mono)'
            }}>
              {errorCount} ERR
            </span>
          )}
        </div>

        {/* Search + subsystem filter */}
        <div className="log-search-row">
          <input
            className="log-search"
            type="text"
            placeholder="Filter messages..."
            value={searchText}
            onChange={e => setSearchText(e.target.value)}
            spellCheck={false}
          />
          <select
            className="log-subsystem-select"
            value={selectedSubsystem}
            onChange={e => setSelectedSubsystem(e.target.value)}
          >
            {subsystems.map(s => (
              <option key={s} value={s}>{s}</option>
            ))}
          </select>
        </div>
      </div>

      {/* Log entries */}
      <div
        className="log-entries"
        ref={containerRef}
        onScroll={handleScroll}
      >
        {filteredLogs.length === 0 && (
          <div style={{
            padding: '24px 16px',
            textAlign: 'center',
            color: 'var(--text-muted)',
            fontSize: '12px'
          }}>
            {logs.length === 0 ? 'No log entries yet.' : 'No entries match the current filters.'}
          </div>
        )}

        {filteredLogs.map((entry, i) => (
          <LogEntry key={entry._id ?? i} entry={entry} />
        ))}

        <div ref={bottomRef} style={{ height: 1 }} />
      </div>

      {/* Footer */}
      <div className="log-footer">
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <span className="log-count-badge">{filteredLogs.length} / {logs.length}</span>
          <span style={{ color: 'var(--text-muted)', fontSize: 10 }}>entries</span>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <label style={{ display: 'flex', alignItems: 'center', gap: 4, cursor: 'pointer', fontSize: 10, color: 'var(--text-dim)' }}>
            <input
              type="checkbox"
              checked={autoScroll}
              onChange={e => setAutoScroll(e.target.checked)}
              style={{ cursor: 'pointer' }}
            />
            Auto-scroll
          </label>
          <button className="btn btn-sm btn-danger" onClick={handleClear}>
            Clear
          </button>
        </div>
      </div>
    </div>
  )
}

const LogEntry = React.memo(function LogEntry({ entry }) {
  const level = entry.level || 'INFO'
  return (
    <div className={`log-entry ${level}`}>
      <span className="log-ts">{formatTimestamp(entry.timestamp)}</span>
      <span className={`log-level-tag ${level}`}>{level}</span>
      {entry.subsystem && (
        <span className="log-subsystem">[{entry.subsystem}]</span>
      )}
      {entry.drone_id != null && (
        <span className="log-drone-id">D{entry.drone_id}</span>
      )}
      <span className={`log-message ${level}`}>{entry.message}</span>
    </div>
  )
})
