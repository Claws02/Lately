import * as THREE from 'three'

/**
 * High-performance instanced drone renderer using THREE.InstancedMesh.
 * Handles up to `maxDrones` drones with per-instance color and transform.
 */
export class InstancedSwarm {
  constructor(scene, maxDrones = 8000) {
    this.scene = scene
    this.maxDrones = maxDrones
    this.currentCount = 0
    this.dummy = new THREE.Object3D()

    this._buildMeshes(maxDrones)
  }

  _buildMeshes(maxDrones) {
    // ---- Drone body (flat box) ----
    this.bodyGeo = new THREE.BoxGeometry(0.3, 0.1, 0.3)
    this.bodyMat = new THREE.MeshStandardMaterial({
      roughness: 0.4,
      metalness: 0.6,
      color: 0xffffff
    })
    this.bodyMesh = new THREE.InstancedMesh(this.bodyGeo, this.bodyMat, maxDrones)
    this.bodyMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage)
    this.bodyMesh.count = 0
    this.bodyMesh.frustumCulled = false
    this.scene.add(this.bodyMesh)

    // ---- LED glow sphere ----
    this.ledGeo = new THREE.SphereGeometry(0.12, 6, 6)
    this.ledMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.85
    })
    this.ledMesh = new THREE.InstancedMesh(this.ledGeo, this.ledMat, maxDrones)
    this.ledMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage)
    this.ledMesh.count = 0
    this.ledMesh.frustumCulled = false
    this.scene.add(this.ledMesh)

    // Ensure color buffers are allocated
    // Force colour buffer allocation by setting count
    this.bodyMesh.setColorAt(0, new THREE.Color(1, 1, 1))
    this.ledMesh.setColorAt(0, new THREE.Color(1, 1, 1))
  }

  /**
   * Re-initialize for a different drone count (e.g. on swarm resize).
   */
  resize(count) {
    if (count === this.maxDrones) return
    this.dispose()
    this.maxDrones = count
    this._buildMeshes(count)
  }

  /**
   * Main update method - called every animation frame with latest drone data.
   * @param {Array} drones - array of drone objects from WebSocket telemetry
   */
  update(drones) {
    if (!drones || drones.length === 0) {
      this.bodyMesh.count = 0
      this.ledMesh.count = 0
      return
    }

    const count = Math.min(drones.length, this.maxDrones)
    const dummy = this.dummy
    const color = new THREE.Color()

    for (let i = 0; i < count; i++) {
      const d = drones[i]

      // Build instance transform
      dummy.position.set(
        typeof d.x === 'number' ? d.x : 0,
        typeof d.y === 'number' ? d.y : 0,
        typeof d.z === 'number' ? d.z : 0
      )
      dummy.rotation.set(
        typeof d.roll === 'number' ? d.roll : 0,
        typeof d.pitch === 'number' ? d.pitch : 0,
        typeof d.yaw === 'number' ? d.yaw : 0
      )
      dummy.scale.setScalar(1)
      dummy.updateMatrix()

      this.bodyMesh.setMatrixAt(i, dummy.matrix)

      // LED position - slightly above body center
      dummy.position.set(
        typeof d.x === 'number' ? d.x : 0,
        (typeof d.y === 'number' ? d.y : 0),
        (typeof d.z === 'number' ? d.z : 0) + 0.08
      )
      dummy.rotation.set(0, 0, 0)
      dummy.updateMatrix()
      this.ledMesh.setMatrixAt(i, dummy.matrix)

      // Per-instance color from LED values
      const r = typeof d.r === 'number' ? d.r / 255 : 1
      const g = typeof d.g === 'number' ? d.g / 255 : 1
      const b = typeof d.b === 'number' ? d.b / 255 : 1

      color.setRGB(r, g, b)
      this.bodyMesh.setColorAt(i, color)
      this.ledMesh.setColorAt(i, color)
    }

    this.bodyMesh.count = count
    this.ledMesh.count = count
    this.currentCount = count

    this.bodyMesh.instanceMatrix.needsUpdate = true
    this.ledMesh.instanceMatrix.needsUpdate = true

    if (this.bodyMesh.instanceColor) this.bodyMesh.instanceColor.needsUpdate = true
    if (this.ledMesh.instanceColor) this.ledMesh.instanceColor.needsUpdate = true
  }

  /**
   * Free GPU resources.
   */
  dispose() {
    if (this.bodyMesh) {
      this.scene.remove(this.bodyMesh)
      this.bodyMesh.dispose()
    }
    if (this.ledMesh) {
      this.scene.remove(this.ledMesh)
      this.ledMesh.dispose()
    }
    if (this.bodyGeo) this.bodyGeo.dispose()
    if (this.ledGeo) this.ledGeo.dispose()
    if (this.bodyMat) this.bodyMat.dispose()
    if (this.ledMat) this.ledMat.dispose()
  }
}
