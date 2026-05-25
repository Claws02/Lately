import * as THREE from 'three'

/**
 * Renders flight path / trajectory lines for selected drones.
 * Each drone gets its own THREE.Line object keyed by drone ID.
 */
export class FlightPathVisualizer {
  constructor(scene) {
    this.scene = scene
    /** @type {Map<number, THREE.Line>} */
    this.lines = new Map()
    this.startTime = Date.now()
  }

  /**
   * Show (or update) a trajectory line for a drone.
   * @param {number} droneId
   * @param {Array<{x: number, y: number, z: number}>} waypoints
   * @param {{ r: number, g: number, b: number }} color - LED color of the drone
   */
  showPath(droneId, waypoints, color = { r: 0, g: 212, b: 255 }) {
    if (!waypoints || waypoints.length < 2) return

    // Remove existing line for this drone
    this.hidePath(droneId)

    const points = waypoints.map(wp =>
      new THREE.Vector3(
        typeof wp.x === 'number' ? wp.x : 0,
        typeof wp.y === 'number' ? wp.y : 0,
        typeof wp.z === 'number' ? wp.z : 0
      )
    )

    const geometry = new THREE.BufferGeometry().setFromPoints(points)
    const material = new THREE.LineBasicMaterial({
      color: new THREE.Color(color.r / 255, color.g / 255, color.b / 255),
      transparent: true,
      opacity: 0.6,
      linewidth: 1  // Note: linewidth > 1 only works with WebGL2 / LineMaterial
    })

    const line = new THREE.Line(geometry, material)
    line.frustumCulled = false
    line.userData.droneId = droneId
    line.userData.createdAt = Date.now()

    this.scene.add(line)
    this.lines.set(droneId, line)
  }

  /**
   * Hide (remove) the flight path line for a specific drone.
   * @param {number} droneId
   */
  hidePath(droneId) {
    const line = this.lines.get(droneId)
    if (line) {
      this.scene.remove(line)
      line.geometry.dispose()
      line.material.dispose()
      this.lines.delete(droneId)
    }
  }

  /**
   * Remove all flight path lines.
   */
  hideAll() {
    for (const [id] of this.lines) {
      this.hidePath(id)
    }
    this.lines.clear()
  }

  /**
   * Update opacity of all lines based on age (fade out over time).
   * Call this each frame for animated fade effect.
   */
  updateFade() {
    const now = Date.now()
    for (const [, line] of this.lines) {
      const age = (now - (line.userData.createdAt || now)) / 1000 // seconds
      const maxAge = 10 // fade out over 10 seconds
      const opacity = Math.max(0, 1 - age / maxAge) * 0.7
      if (line.material) {
        line.material.opacity = opacity
      }
      if (age > maxAge) {
        this.hidePath(line.userData.droneId)
      }
    }
  }

  /**
   * Clean up all resources.
   */
  dispose() {
    this.hideAll()
  }
}
