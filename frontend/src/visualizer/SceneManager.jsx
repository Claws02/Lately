import React, { useEffect, useRef, useCallback, useState } from 'react'
import * as THREE from 'three'
import { useSimStore } from '../store'
import { InstancedSwarm } from './InstancedSwarm'
import { FlightPathVisualizer } from './FlightPath'

// ─────────────────────────────────────────────
// Minimal OrbitControls implemented inline
// ─────────────────────────────────────────────
class MinimalOrbitControls {
  constructor(camera, domElement) {
    this.camera = camera
    this.domElement = domElement
    this.enabled = true

    // Spherical coordinates
    this.spherical = new THREE.Spherical()
    this.spherical.setFromVector3(camera.position)
    this.target = new THREE.Vector3(0, 0, 20)

    this.minDistance = 5
    this.maxDistance = 800
    this.minPolarAngle = 0.05
    this.maxPolarAngle = Math.PI * 0.9

    this._isDragging = false
    this._isPanning = false
    this._lastMouse = { x: 0, y: 0 }
    this._rotateSpeed = 0.005
    this._panSpeed = 0.15
    this._zoomSpeed = 1.1

    this._onMouseDown = this._onMouseDown.bind(this)
    this._onMouseMove = this._onMouseMove.bind(this)
    this._onMouseUp = this._onMouseUp.bind(this)
    this._onWheel = this._onWheel.bind(this)
    this._onTouchStart = this._onTouchStart.bind(this)
    this._onTouchMove = this._onTouchMove.bind(this)
    this._onTouchEnd = this._onTouchEnd.bind(this)
    this._touchStartDist = 0

    domElement.addEventListener('mousedown', this._onMouseDown)
    domElement.addEventListener('mousemove', this._onMouseMove)
    domElement.addEventListener('mouseup', this._onMouseUp)
    domElement.addEventListener('mouseleave', this._onMouseUp)
    domElement.addEventListener('wheel', this._onWheel, { passive: false })
    domElement.addEventListener('touchstart', this._onTouchStart, { passive: false })
    domElement.addEventListener('touchmove', this._onTouchMove, { passive: false })
    domElement.addEventListener('touchend', this._onTouchEnd)
    domElement.addEventListener('contextmenu', e => e.preventDefault())
  }

  _onMouseDown(e) {
    if (!this.enabled) return
    this._isDragging = e.button === 0
    this._isPanning = e.button === 2
    this._lastMouse = { x: e.clientX, y: e.clientY }
  }

  _onMouseMove(e) {
    if (!this.enabled) return
    const dx = e.clientX - this._lastMouse.x
    const dy = e.clientY - this._lastMouse.y
    this._lastMouse = { x: e.clientX, y: e.clientY }

    if (this._isDragging) {
      this.spherical.theta -= dx * this._rotateSpeed
      this.spherical.phi -= dy * this._rotateSpeed
      this.spherical.phi = Math.max(this.minPolarAngle, Math.min(this.maxPolarAngle, this.spherical.phi))
      this._updateCamera()
    } else if (this._isPanning) {
      const right = new THREE.Vector3()
      const up = new THREE.Vector3()
      right.crossVectors(
        new THREE.Vector3().subVectors(this.camera.position, this.target).normalize(),
        this.camera.up
      ).negate().normalize()
      up.copy(this.camera.up).normalize()
      const panScale = this.spherical.radius * this._panSpeed * 0.01
      this.target.addScaledVector(right, -dx * panScale)
      this.target.addScaledVector(up, dy * panScale)
      this._updateCamera()
    }
  }

  _onMouseUp() {
    this._isDragging = false
    this._isPanning = false
  }

  _onWheel(e) {
    if (!this.enabled) return
    e.preventDefault()
    const delta = e.deltaY > 0 ? this._zoomSpeed : 1 / this._zoomSpeed
    this.spherical.radius = Math.max(
      this.minDistance,
      Math.min(this.maxDistance, this.spherical.radius * delta)
    )
    this._updateCamera()
  }

  _onTouchStart(e) {
    if (!this.enabled) return
    e.preventDefault()
    if (e.touches.length === 1) {
      this._isDragging = true
      this._lastMouse = { x: e.touches[0].clientX, y: e.touches[0].clientY }
    } else if (e.touches.length === 2) {
      this._isDragging = false
      const dx = e.touches[0].clientX - e.touches[1].clientX
      const dy = e.touches[0].clientY - e.touches[1].clientY
      this._touchStartDist = Math.sqrt(dx * dx + dy * dy)
    }
  }

  _onTouchMove(e) {
    if (!this.enabled) return
    e.preventDefault()
    if (e.touches.length === 1 && this._isDragging) {
      const dx = e.touches[0].clientX - this._lastMouse.x
      const dy = e.touches[0].clientY - this._lastMouse.y
      this._lastMouse = { x: e.touches[0].clientX, y: e.touches[0].clientY }
      this.spherical.theta -= dx * this._rotateSpeed
      this.spherical.phi -= dy * this._rotateSpeed
      this.spherical.phi = Math.max(this.minPolarAngle, Math.min(this.maxPolarAngle, this.spherical.phi))
      this._updateCamera()
    } else if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX
      const dy = e.touches[0].clientY - e.touches[1].clientY
      const dist = Math.sqrt(dx * dx + dy * dy)
      const ratio = this._touchStartDist / dist
      this.spherical.radius = Math.max(
        this.minDistance,
        Math.min(this.maxDistance, this.spherical.radius * ratio)
      )
      this._touchStartDist = dist
      this._updateCamera()
    }
  }

  _onTouchEnd() {
    this._isDragging = false
  }

  _updateCamera() {
    this.spherical.makeSafe()
    const pos = new THREE.Vector3().setFromSpherical(this.spherical)
    this.camera.position.copy(pos.add(this.target))
    this.camera.lookAt(this.target)
  }

  setTarget(x, y, z) {
    this.target.set(x, y, z)
    this._updateCamera()
  }

  setPosition(r, phi, theta) {
    this.spherical.set(r, phi, theta)
    this._updateCamera()
  }

  dispose() {
    this.domElement.removeEventListener('mousedown', this._onMouseDown)
    this.domElement.removeEventListener('mousemove', this._onMouseMove)
    this.domElement.removeEventListener('mouseup', this._onMouseUp)
    this.domElement.removeEventListener('mouseleave', this._onMouseUp)
    this.domElement.removeEventListener('wheel', this._onWheel)
    this.domElement.removeEventListener('touchstart', this._onTouchStart)
    this.domElement.removeEventListener('touchmove', this._onTouchMove)
    this.domElement.removeEventListener('touchend', this._onTouchEnd)
  }
}

// ─────────────────────────────────────────────
// SimulatorCanvas React Component
// ─────────────────────────────────────────────
export function SimulatorCanvas() {
  const mountRef = useRef(null)
  const rendererRef = useRef(null)
  const sceneRef = useRef(null)
  const cameraRef = useRef(null)
  const controlsRef = useRef(null)
  const swarmRef = useRef(null)
  const flightPathRef = useRef(null)
  const animFrameRef = useRef(null)
  const fpsRef = useRef({ frames: 0, last: performance.now(), fps: 0 })
  const dronesRef = useRef([])

  const [fps, setFps] = useState(0)
  const [droneCount, setDroneCount] = useState(0)

  const drones = useSimStore(state => state.drones)
  const simState = useSimStore(state => state.simState)

  // Keep dronesRef in sync for the animation loop (avoids closure stale reference)
  useEffect(() => {
    dronesRef.current = drones
    setDroneCount(drones.length)
  }, [drones])

  // Camera preset functions exposed via callbacks
  const setCameraPreset = useCallback((preset) => {
    const controls = controlsRef.current
    if (!controls) return
    switch (preset) {
      case 'top':
        controls.spherical.set(200, 0.01, 0)
        controls.setTarget(0, 0, 20)
        break
      case 'side':
        controls.spherical.set(200, Math.PI / 2, Math.PI / 2)
        controls.setTarget(0, 0, 20)
        break
      case 'audience':
        controls.spherical.set(180, 1.1, -Math.PI / 2)
        controls.setTarget(0, 0, 30)
        break
      case 'iso':
      default:
        controls.spherical.set(150, Math.PI / 4, -Math.PI / 4)
        controls.setTarget(0, 0, 20)
        break
    }
    controls._updateCamera()
  }, [])

  useEffect(() => {
    const container = mountRef.current
    if (!container) return

    // ── Renderer ──
    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      logarithmicDepthBuffer: true,
      powerPreference: 'high-performance'
    })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setSize(container.clientWidth, container.clientHeight)
    renderer.setClearColor(0x0a0e1a, 1)
    renderer.shadowMap.enabled = false
    container.appendChild(renderer.domElement)
    rendererRef.current = renderer

    // ── Scene ──
    const scene = new THREE.Scene()
    scene.fog = new THREE.FogExp2(0x0a0e1a, 0.003)
    sceneRef.current = scene

    // ── Camera ──
    const camera = new THREE.PerspectiveCamera(
      60,
      container.clientWidth / container.clientHeight,
      0.1,
      5000
    )
    camera.position.set(0, -100, 80)
    camera.lookAt(0, 0, 0)
    cameraRef.current = camera

    // ── Lights ──
    const hemiLight = new THREE.HemisphereLight(0x334466, 0x222222, 0.6)
    scene.add(hemiLight)

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.3)
    dirLight.position.set(50, 50, 100)
    scene.add(dirLight)

    const ambientLight = new THREE.AmbientLight(0x0a0e1a, 0.5)
    scene.add(ambientLight)

    // ── Grid ──
    const gridHelper = new THREE.GridHelper(200, 20, 0x1a2744, 0x1a2744)
    gridHelper.position.y = 0
    // Three.js GridHelper is in XZ plane by default; rotate to XY for Z-up feel
    scene.add(gridHelper)

    // Ground plane (semi-transparent)
    const groundGeo = new THREE.PlaneGeometry(400, 400)
    const groundMat = new THREE.MeshBasicMaterial({
      color: 0x050a14,
      transparent: true,
      opacity: 0.6,
      side: THREE.FrontSide
    })
    const ground = new THREE.Mesh(groundGeo, groundMat)
    ground.rotation.x = -Math.PI / 2
    ground.position.y = -0.05
    scene.add(ground)

    // ── Axes helper (small) ──
    const axesHelper = new THREE.AxesHelper(5)
    scene.add(axesHelper)

    // ── Origin marker ──
    const originGeo = new THREE.SphereGeometry(0.5, 8, 8)
    const originMat = new THREE.MeshBasicMaterial({ color: 0xffffff, opacity: 0.5, transparent: true })
    const originMesh = new THREE.Mesh(originGeo, originMat)
    scene.add(originMesh)

    // ── Controls ──
    const controls = new MinimalOrbitControls(camera, renderer.domElement)
    controls.setPosition(150, Math.PI / 4, -Math.PI / 4)
    controls.setTarget(0, 0, 20)
    controlsRef.current = controls

    // ── Swarm renderer ──
    const swarm = new InstancedSwarm(scene, 8000)
    swarmRef.current = swarm

    // ── Flight path visualizer ──
    const flightPath = new FlightPathVisualizer(scene)
    flightPathRef.current = flightPath

    // ── Animation loop ──
    let running = true
    const clock = new THREE.Clock()

    const animate = () => {
      if (!running) return
      animFrameRef.current = requestAnimationFrame(animate)

      // FPS tracking
      const now = performance.now()
      fpsRef.current.frames++
      if (now - fpsRef.current.last >= 500) {
        const elapsed = (now - fpsRef.current.last) / 1000
        fpsRef.current.fps = Math.round(fpsRef.current.frames / elapsed)
        fpsRef.current.frames = 0
        fpsRef.current.last = now
        setFps(fpsRef.current.fps)
      }

      // Update swarm with latest drone data
      if (swarmRef.current && dronesRef.current) {
        swarmRef.current.update(dronesRef.current)
      }

      // Update flight path fade
      if (flightPathRef.current) {
        flightPathRef.current.updateFade()
      }

      renderer.render(scene, camera)
    }

    animate()

    // ── Resize handler ──
    const handleResize = () => {
      if (!container || !renderer || !camera) return
      const w = container.clientWidth
      const h = container.clientHeight
      camera.aspect = w / h
      camera.updateProjectionMatrix()
      renderer.setSize(w, h)
    }

    const resizeObserver = new ResizeObserver(handleResize)
    resizeObserver.observe(container)

    // ── Cleanup ──
    return () => {
      running = false
      if (animFrameRef.current) cancelAnimationFrame(animFrameRef.current)
      resizeObserver.disconnect()
      controls.dispose()
      swarm.dispose()
      flightPath.dispose()
      renderer.dispose()
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement)
      }
    }
  }, [])

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      {/* Three.js canvas mount point */}
      <div
        ref={mountRef}
        style={{ width: '100%', height: '100%' }}
      />

      {/* FPS + drone count - top left */}
      <div className="canvas-overlay canvas-overlay-tl">
        <div className="fps-counter">
          <span>{fps}</span> fps &nbsp;|&nbsp; <span>{droneCount}</span> drones
        </div>
      </div>

      {/* Camera presets - top right */}
      <div className="canvas-overlay canvas-overlay-tr">
        <div className="camera-presets">
          <button className="camera-preset-btn" onClick={() => setCameraPreset('iso')}>Isometric</button>
          <button className="camera-preset-btn" onClick={() => setCameraPreset('top')}>Top View</button>
          <button className="camera-preset-btn" onClick={() => setCameraPreset('side')}>Side View</button>
          <button className="camera-preset-btn" onClick={() => setCameraPreset('audience')}>Audience View</button>
        </div>
      </div>

      {/* Show status HUD - bottom left */}
      <div className="canvas-overlay canvas-overlay-bl">
        <div className="show-status-hud">
          <div className="hud-title">Show Status</div>
          <div className="hud-row">
            <span className="hud-row-label">State</span>
            <span className={`hud-row-value ${simState.running ? 'running' : 'stopped'}`}>
              {simState.running ? 'RUNNING' : 'STOPPED'}
            </span>
          </div>
          <div className="hud-row">
            <span className="hud-row-label">Time</span>
            <span className="hud-row-value">
              {formatTime(simState.showTime || 0)}
            </span>
          </div>
          <div className="hud-row">
            <span className="hud-row-label">Duration</span>
            <span className="hud-row-value">
              {formatTime(simState.showDuration || 0)}
            </span>
          </div>
          <div className="hud-row">
            <span className="hud-row-label">Sim Hz</span>
            <span className="hud-row-value">
              {(simState.simHz || 0).toFixed(1)}
            </span>
          </div>
          <div className="hud-row">
            <span className="hud-row-label">RTF</span>
            <span className="hud-row-value">
              {(simState.realTimeFactor || 1).toFixed(2)}x
            </span>
          </div>
        </div>
      </div>

      {/* Altitude legend - bottom right */}
      <div className="canvas-overlay canvas-overlay-br">
        <div className="altitude-legend">
          <div className="altitude-legend-title">Altitude</div>
          <div className="altitude-legend-inner">
            <div className="altitude-bar" />
            <div className="altitude-labels">
              <span>100m+</span>
              <span>75m</span>
              <span>50m</span>
              <span>25m</span>
              <span>0m</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

function formatTime(seconds) {
  if (!seconds || isNaN(seconds)) return '00:00'
  const m = Math.floor(seconds / 60)
  const s = Math.floor(seconds % 60)
  return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`
}
