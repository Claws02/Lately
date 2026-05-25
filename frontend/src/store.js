import { create } from 'zustand'

export const useSimStore = create((set, get) => ({
  drones: [],
  simState: {
    running: false,
    showLoaded: false,
    showTime: 0,
    showDuration: 0,
    droneCount: 0,
    activeDrones: 0,
    failsafeDrones: 0,
    simHz: 0,
    realTimeFactor: 1.0
  },
  isConnected: false,
  logs: [],
  showInfo: null,

  setDrones: (drones) => set({ drones }),

  updateDrone: (id, data) => set(state => ({
    drones: state.drones.map(d => d.id === id ? { ...d, ...data } : d)
  })),

  setSimState: (simState) => set(state => ({
    simState: { ...state.simState, ...simState }
  })),

  setConnected: (isConnected) => set({ isConnected }),

  addLog: (entry) => set(state => ({
    logs: [...state.logs.slice(-499), entry]
  })),

  clearLogs: () => set({ logs: [] }),

  setShowInfo: (showInfo) => set({ showInfo }),

  getDroneStats: () => {
    const { drones } = get()
    if (!drones.length) {
      return {
        total: 0,
        flying: 0,
        idle: 0,
        takeoff: 0,
        landing: 0,
        failsafe: 0,
        arming: 0,
        grounded: 0,
        other: 0,
        avgBattery: 0,
        minBattery: 0,
        rtkFixCount: 0,
        rtkFixPct: 0
      }
    }
    const flying = drones.filter(d => d.status === 'FLYING').length
    const idle = drones.filter(d => d.status === 'IDLE').length
    const takeoff = drones.filter(d => d.status === 'TAKEOFF').length
    const landing = drones.filter(d => d.status === 'LANDING').length
    const failsafe = drones.filter(d => d.status === 'FAILSAFE').length
    const arming = drones.filter(d => d.status === 'ARMING').length
    const grounded = drones.filter(d => d.status === 'GROUNDED').length
    const other = drones.length - flying - idle - takeoff - landing - failsafe - arming - grounded
    const batteries = drones.map(d => d.battery ?? 100)
    const avgBattery = batteries.reduce((s, v) => s + v, 0) / batteries.length
    const minBattery = Math.min(...batteries)
    const rtkFixCount = drones.filter(d => d.rtk_fix).length
    const rtkFixPct = (rtkFixCount / drones.length) * 100
    return {
      total: drones.length,
      flying,
      idle,
      takeoff,
      landing,
      failsafe,
      arming,
      grounded,
      other: Math.max(0, other),
      avgBattery,
      minBattery,
      rtkFixCount,
      rtkFixPct
    }
  }
}))
