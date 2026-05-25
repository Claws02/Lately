/**
 * WebSocket client with auto-reconnect, message queuing, and heartbeat.
 */
export class WebSocketClient {
  constructor(url, onMessage, onConnect, onDisconnect) {
    this.url = url
    this.onMessage = onMessage
    this.onConnect = onConnect
    this.onDisconnect = onDisconnect

    this.ws = null
    this.isConnected = false
    this.reconnectAttempts = 0
    this.maxReconnectAttempts = 10
    this.reconnectTimer = null
    this.heartbeatTimer = null
    this.messageQueue = []
    this.destroyed = false
    this.manualClose = false

    // Exponential backoff delays in ms: 1s, 2s, 4s, 8s, 16s, then cap at 30s
    this.backoffDelays = [1000, 2000, 4000, 8000, 16000, 30000]
  }

  connect() {
    if (this.destroyed) return
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return
    }
    this.manualClose = false

    try {
      this.ws = new WebSocket(this.url)
    } catch (err) {
      console.error('[WS] Failed to create WebSocket:', err)
      this._scheduleReconnect()
      return
    }

    this.ws.onopen = () => {
      if (this.destroyed) {
        this.ws.close()
        return
      }
      console.log('[WS] Connected to', this.url)
      this.isConnected = true
      this.reconnectAttempts = 0

      // Flush queued messages
      while (this.messageQueue.length > 0) {
        const msg = this.messageQueue.shift()
        try {
          this.ws.send(msg)
        } catch (e) {
          console.warn('[WS] Failed to send queued message:', e)
        }
      }

      this._startHeartbeat()
      if (this.onConnect) this.onConnect()
    }

    this.ws.onmessage = (event) => {
      if (this.destroyed) return
      try {
        const data = JSON.parse(event.data)
        if (this.onMessage) this.onMessage(data)
      } catch (err) {
        console.warn('[WS] Failed to parse message:', err)
      }
    }

    this.ws.onclose = (event) => {
      this.isConnected = false
      this._stopHeartbeat()
      console.log(`[WS] Disconnected (code=${event.code}, clean=${event.wasClean})`)
      if (this.onDisconnect) this.onDisconnect()
      if (!this.manualClose && !this.destroyed) {
        this._scheduleReconnect()
      }
    }

    this.ws.onerror = (err) => {
      console.warn('[WS] Error:', err)
      // onclose will fire after onerror, so reconnect is handled there
    }
  }

  disconnect() {
    this.manualClose = true
    this.destroyed = true
    this._stopHeartbeat()
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    if (this.ws) {
      try { this.ws.close(1000, 'Client disconnect') } catch (e) { /* ignore */ }
      this.ws = null
    }
    this.isConnected = false
  }

  send(data) {
    const msg = typeof data === 'string' ? data : JSON.stringify(data)
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      try {
        this.ws.send(msg)
      } catch (e) {
        console.warn('[WS] Send failed:', e)
        this.messageQueue.push(msg)
      }
    } else {
      // Queue for when connection opens (limit queue size)
      if (this.messageQueue.length < 50) {
        this.messageQueue.push(msg)
      }
    }
  }

  _scheduleReconnect() {
    if (this.destroyed || this.manualClose) return
    if (this.reconnectTimer) return // already scheduled

    const idx = Math.min(this.reconnectAttempts, this.backoffDelays.length - 1)
    const delay = this.backoffDelays[idx]
    this.reconnectAttempts++

    console.log(`[WS] Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`)
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null
      if (!this.destroyed && !this.manualClose) {
        this.connect()
      }
    }, delay)
  }

  _startHeartbeat() {
    this._stopHeartbeat()
    this.heartbeatTimer = setInterval(() => {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        try {
          // Send a ping-style keepalive
          this.ws.send(JSON.stringify({ type: 'PING', ts: Date.now() }))
        } catch (e) {
          console.warn('[WS] Heartbeat send failed:', e)
        }
      }
    }, 30000)
  }

  _stopHeartbeat() {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer)
      this.heartbeatTimer = null
    }
  }

  get readyState() {
    return this.ws ? this.ws.readyState : WebSocket.CLOSED
  }
}
