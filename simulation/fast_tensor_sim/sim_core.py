"""
NextGen Swarm Simulator - Vectorized Kinematic Physics Engine
Simulates 8000+ drones in real-time using numpy tensor operations.
All drone states stored as flat numpy arrays for vectorized computation.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Dict, Tuple

import numpy as np

from .drone_math import (
    BatteryModel,
    LBAPAssigner,
    OrcaCollisionAvoider,
    TrajectoryInterpolator,
    WindModel,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Enums & Dataclasses
# ---------------------------------------------------------------------------


class DroneStatus(IntEnum):
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    FLYING = 3
    LANDING = 4
    GROUNDED = 5
    FAILSAFE = 6
    ERROR = 7


@dataclass
class DroneState:
    id: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    r: int
    g: int
    b: int
    battery: float
    status: DroneStatus
    rtk_fix: bool


@dataclass
class SimulatorState:
    running: bool
    show_loaded: bool
    show_time: float
    show_duration: float
    drone_count: int
    active_drones: int
    failsafe_drones: int
    sim_hz: float
    real_time_factor: float


# ---------------------------------------------------------------------------
# LightInterpolator
# ---------------------------------------------------------------------------


class LightInterpolator:
    """
    Linear interpolation of RGB LED keyframes.

    Each keyframe dict: {t, r, g, b}
    """

    def __init__(self, keyframes: List[Dict]):
        if not keyframes:
            keyframes = [{"t": 0.0, "r": 255, "g": 255, "b": 255}]

        kfs = sorted(keyframes, key=lambda k: k["t"])
        self._times = np.array([k["t"] for k in kfs], dtype=np.float64)
        self._colors = np.array(
            [[k["r"], k["g"], k["b"]] for k in kfs], dtype=np.float64
        )

    def color_at(self, t: float) -> Tuple[int, int, int]:
        """Linear interpolation between keyframes."""
        t = float(np.clip(t, self._times[0], self._times[-1]))

        idx = int(np.searchsorted(self._times, t, side="right")) - 1
        idx = int(np.clip(idx, 0, len(self._times) - 2))

        t0 = self._times[idx]
        t1 = self._times[idx + 1]
        span = t1 - t0
        u = float((t - t0) / span) if span > 1e-12 else 0.0
        u = float(np.clip(u, 0.0, 1.0))

        c0 = self._colors[idx]
        c1 = self._colors[idx + 1]
        blended = c0 + u * (c1 - c0)
        return (
            int(np.clip(round(blended[0]), 0, 255)),
            int(np.clip(round(blended[1]), 0, 255)),
            int(np.clip(round(blended[2]), 0, 255)),
        )


# ---------------------------------------------------------------------------
# Physics Constants
# ---------------------------------------------------------------------------

MAX_VELOCITY = 8.0          # m/s
MAX_ACCELERATION = 5.0      # m/s²
MAX_JERK = 10.0             # m/s³
HOVER_ALT = 10.0            # m — drones hover here when idle
TAKEOFF_SPEED = 2.0         # m/s
LANDING_SPEED = 1.5         # m/s
MIN_DRONE_SEPARATION = 1.5  # m
KP_POS = 2.5                # position P gain
KD_VEL = 1.8                # velocity D gain
SIM_DT = 0.01               # s  (100 Hz physics)
TELEMETRY_DT = 1.0 / 60.0  # s  (60 Hz telemetry)

# Geofence limits (metres from origin)
GEOFENCE_RADIUS = 500.0
GEOFENCE_MAX_ALT = 120.0
GEOFENCE_MIN_ALT = -1.0     # slightly below ground

# Arming delay before TAKEOFF begins
ARMING_DELAY_S = 1.0


# ---------------------------------------------------------------------------
# Dict adapter — bridges generate_demo_show() raw dicts to ShowFile interface
# ---------------------------------------------------------------------------

class _DictShowFileAdapter:
    """Wraps the dict returned by demo_generator.generate_demo_show() so that
    DroneSwarmSimulator.load_show() can consume it without modification."""

    def __init__(self, d: dict) -> None:
        self._d = d
        meta = d.get("metadata", {})
        self.drone_count: int = int(meta.get("drone_count", len(d.get("trajectories", []))))
        self.title: str = meta.get("title", "")
        self.duration: float = float(meta.get("duration", 0.0))

    def get_trajectories(self) -> list:
        """Return list of {waypoints, lights} dicts the simulator expects."""
        trajs = {t["drone_id"]: t["waypoints"] for t in self._d.get("trajectories", [])}
        lights = {lc["drone_id"]: lc["keyframes"] for lc in self._d.get("light_cues", [])}
        result = []
        for i in range(self.drone_count):
            result.append({
                "waypoints": trajs.get(i, [{"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}]),
                "lights": lights.get(i, [{"t": 0.0, "r": 255, "g": 255, "b": 255}]),
            })
        return result


# ---------------------------------------------------------------------------
# DroneSwarmSimulator
# ---------------------------------------------------------------------------


class DroneSwarmSimulator:
    """
    Vectorised kinematic drone swarm simulator.

    State is stored as flat numpy arrays (shape (N,) or (N,3)) for maximum
    throughput via SIMD / numpy vectorisation.  The physics loop runs at
    SIM_DT (100 Hz) and telemetry is gathered at TELEMETRY_DT (60 Hz).
    """

    def __init__(self, drone_count: int = 100):
        self._drone_count = 0           # will be set by set_drone_count
        self._running = False
        self._show_time = 0.0
        self._show_duration = 0.0
        self._show_loaded = False
        self._show_playing = False

        # Subsystems
        self._orca = OrcaCollisionAvoider(
            min_distance=MIN_DRONE_SEPARATION,
            time_horizon=3.0,
            max_speed=MAX_VELOCITY,
        )
        self._battery_model = BatteryModel()
        self._wind_model = WindModel(speed=0.0, turbulence=0.0)
        self._assigner = LBAPAssigner()

        # Trajectory / light data
        self.trajectories: List[Optional[TrajectoryInterpolator]] = []
        self.light_interpolators: List[Optional[LightInterpolator]] = []

        # Performance tracking
        self._step_count = 0
        self._last_telemetry_time = 0.0
        self._real_time_factor = 1.0
        self._sim_hz_actual = 0.0
        self._collision_count = 0
        self._failsafe_count = 0

        # Arming timer (per-drone, seconds)
        self._arming_elapsed: Optional[np.ndarray] = None

        # Initialise arrays for requested drone count
        self.set_drone_count(drone_count)

    # ------------------------------------------------------------------
    # Array initialisation / resizing
    # ------------------------------------------------------------------

    def set_drone_count(self, n: int):
        """Resize all simulation arrays for *n* drones."""
        old_n = self._drone_count
        self._drone_count = n

        # ---- State arrays ----
        self.positions = np.zeros((n, 3), dtype=np.float64)
        self.velocities = np.zeros((n, 3), dtype=np.float64)
        self.accelerations = np.zeros((n, 3), dtype=np.float64)
        self.orientations = np.zeros((n, 3), dtype=np.float64)   # roll, pitch, yaw
        self.colors = np.full((n, 3), 255, dtype=np.uint8)        # default white
        self.battery = np.full(n, 100.0, dtype=np.float64)
        self.status = np.full(n, DroneStatus.IDLE, dtype=np.int32)
        self.rtk_fix = np.ones(n, dtype=bool)

        self.target_positions = np.zeros((n, 3), dtype=np.float64)
        self.target_velocities = np.zeros((n, 3), dtype=np.float64)

        self._arming_elapsed = np.zeros(n, dtype=np.float64)

        # Trajectory/light lists
        self.trajectories = [None] * n
        self.light_interpolators = [None] * n

        # Place drones on the ground
        self.initialize_formation()

    def initialize_formation(self):
        """
        Place all drones in a rectangular grid at z = 0, spaced 2 m apart.
        The grid is centred on the origin.
        """
        n = self._drone_count
        cols = math.ceil(math.sqrt(n))
        spacing = 2.0

        for i in range(n):
            row = i // cols
            col = i % cols
            # Centre the grid
            x = (col - (cols - 1) / 2.0) * spacing
            y = (row - (math.ceil(n / cols) - 1) / 2.0) * spacing
            self.positions[i] = [x, y, 0.0]
            self.target_positions[i] = [x, y, HOVER_ALT]

    # ------------------------------------------------------------------
    # Show loading
    # ------------------------------------------------------------------

    def load_show(self, show_file) -> bool:
        """
        Load a ShowFile and assign trajectories to drones using LBAP.

        Expected ShowFile interface:
          show_file.drone_count          – int
          show_file.get_trajectories()   – list of dicts {waypoints, lights}
            waypoints: list[{t, x, y, z}]
            lights:    list[{t, r, g, b}]

        Returns True on success.

        Also accepts the raw dict format produced by demo_generator.generate_demo_show():
          {"metadata": {"drone_count": N, ...}, "trajectories": [...], "light_cues": [...]}
        """
        # Normalise raw dict (from demo_generator) into ShowFile-compatible object
        if isinstance(show_file, dict):
            show_file = _DictShowFileAdapter(show_file)

        try:
            show_drone_count = int(show_file.drone_count)

            if show_drone_count != self._drone_count:
                self.set_drone_count(show_drone_count)

            show_tracks = show_file.get_trajectories()

            # Build initial show positions (t=0) for LBAP assignment
            show_initial_positions = np.zeros((show_drone_count, 3), dtype=np.float64)
            for idx, track in enumerate(show_tracks):
                wps = track["waypoints"]
                t0_wp = min(wps, key=lambda w: w["t"])
                show_initial_positions[idx] = [t0_wp["x"], t0_wp["y"], t0_wp["z"]]

            # Assign physical drones to show slots
            assignment = self._assigner.assign(
                self.positions[:, :3],
                show_initial_positions,
                method="lbap",
            )

            # Build interpolators
            max_duration = 0.0
            for phys_i, show_j in enumerate(assignment):
                track = show_tracks[show_j]
                interp = TrajectoryInterpolator(track["waypoints"])
                self.trajectories[phys_i] = interp
                max_duration = max(max_duration, interp.duration())

                lights = track.get("lights", [{"t": 0.0, "r": 255, "g": 255, "b": 255}])
                self.light_interpolators[phys_i] = LightInterpolator(lights)

            self._show_duration = max_duration
            self._show_time = 0.0
            self._show_loaded = True
            self._show_playing = False

            logger.info(
                "Show loaded: %d drones, %.1f s duration.", show_drone_count, max_duration
            )
            return True

        except Exception as exc:
            logger.error("Failed to load show: %s", exc, exc_info=True)
            return False

    # ------------------------------------------------------------------
    # Main async loop
    # ------------------------------------------------------------------

    async def run(self):
        """
        Main async simulation loop at SIM_DT (100 Hz).

        Drives physics at the requested rate and tracks real_time_factor
        (how closely the simulation matches wall-clock time).
        """
        self._running = True
        self._last_telemetry_time = 0.0
        loop_interval = SIM_DT

        logger.info("Simulator started (%d drones, %.0f Hz).", self._drone_count, 1.0 / SIM_DT)

        while self._running:
            t_loop_start = time.perf_counter()

            # --- Physics ---
            self._physics_step()
            self._step_count += 1

            # --- Telemetry ---
            sim_t = self._step_count * SIM_DT
            if sim_t - self._last_telemetry_time >= TELEMETRY_DT:
                self._last_telemetry_time = sim_t
                # Telemetry is available via get_telemetry(); callers poll it.

            # --- Real-time pacing ---
            elapsed = time.perf_counter() - t_loop_start
            sleep_time = loop_interval - elapsed
            if sleep_time > 0.0:
                await asyncio.sleep(sleep_time)
                actual_elapsed = loop_interval
            else:
                actual_elapsed = elapsed
                await asyncio.sleep(0)   # yield control

            # Track real-time factor (exponential moving average)
            rtf = loop_interval / max(actual_elapsed, 1e-9)
            self._real_time_factor = 0.95 * self._real_time_factor + 0.05 * rtf
            self._sim_hz_actual = 0.95 * self._sim_hz_actual + 0.05 * (1.0 / max(actual_elapsed, 1e-9))

    def stop(self):
        """Stop the simulation loop."""
        self._running = False

    # ------------------------------------------------------------------
    # Core physics step
    # ------------------------------------------------------------------

    def _physics_step(self):
        """
        Single physics timestep — the core engine.

        Phases
        ------
        1.  Update show time and pull trajectory targets.
        2.  PD controller desired accelerations.
        3.  ORCA collision avoidance (modifies velocities).
        4.  Wind perturbations.
        5.  Clamp accelerations to MAX_ACCELERATION.
        6.  Euler integration of velocity and position.
        7.  Update orientations (lean into velocity direction).
        8.  Update LED colours from light cues.
        9.  Battery drain.
        10. Geofence violation → FAILSAFE.
        11. Takeoff / landing state machine transitions.
        """
        dt = SIM_DT

        # --- Phase 1: show time ---
        if self._show_playing and self._show_loaded:
            self._show_time = min(self._show_time + dt, self._show_duration)
            self._update_targets_from_show()
            if self._show_time >= self._show_duration:
                self._show_playing = False
                logger.info("Show finished.")

        # --- Phase 11 first pass: takeoff/landing state machines ---
        self._handle_takeoff()
        self._handle_landing()

        # --- Phase 2: PD acceleration ---
        desired_acc = self._compute_pd_acceleration()  # (N, 3)

        # --- Phase 3: ORCA (only for FLYING drones) ---
        flying_mask = (self.status == DroneStatus.FLYING)
        if np.any(flying_mask):
            flying_idx = np.where(flying_mask)[0]
            positions_flying = self.positions[flying_idx]
            velocities_flying = self.velocities[flying_idx]
            pref_vel_flying = self.velocities[flying_idx] + desired_acc[flying_idx] * dt

            new_vel_flying = self._orca.compute_new_velocities(
                positions_flying, velocities_flying, pref_vel_flying
            )
            # Back-compute adjusted acceleration
            desired_acc[flying_idx] = (new_vel_flying - velocities_flying) / dt

        # --- Phase 4: wind perturbations (convert wind force to acc perturbation) ---
        # Treat wind_force as m/s drift added to acceleration (lightweight model)
        wind_forces = self._wind_model.get_force(self.positions, self._show_time)  # (N, 3)
        # Scale by a drag coefficient to convert velocity perturbation to acceleration
        wind_acc = wind_forces * 0.05  # empirical drag factor
        desired_acc += wind_acc

        # --- Phase 5: clamp accelerations ---
        acc_magnitudes = np.linalg.norm(desired_acc, axis=1, keepdims=True)
        over_mask = acc_magnitudes[:, 0] > MAX_ACCELERATION
        if np.any(over_mask):
            scale = np.where(
                over_mask,
                MAX_ACCELERATION / np.maximum(acc_magnitudes[:, 0], 1e-12),
                1.0,
            )
            desired_acc *= scale[:, np.newaxis]

        # Zero acceleration for grounded / error / failsafe drones
        inactive = np.isin(self.status, [DroneStatus.GROUNDED, DroneStatus.ERROR, DroneStatus.FAILSAFE])
        desired_acc[inactive] = 0.0

        self.accelerations = desired_acc

        # --- Phase 6: Euler integration ---
        self.velocities += desired_acc * dt

        # Clamp velocity magnitude
        vel_mag = np.linalg.norm(self.velocities, axis=1, keepdims=True)
        over_vel = vel_mag[:, 0] > MAX_VELOCITY
        if np.any(over_vel):
            scale_v = np.where(
                over_vel,
                MAX_VELOCITY / np.maximum(vel_mag[:, 0], 1e-12),
                1.0,
            )
            self.velocities *= scale_v[:, np.newaxis]

        self.positions += self.velocities * dt

        # Keep grounded drones at z >= 0
        grounded_mask = (self.status == DroneStatus.GROUNDED)
        self.positions[grounded_mask, 2] = np.maximum(self.positions[grounded_mask, 2], 0.0)
        self.velocities[grounded_mask] = 0.0

        # --- Phase 7: orientations ---
        self._update_orientations()

        # --- Phase 8: LED colours ---
        self._update_colors()

        # --- Phase 9: battery ---
        drain = self._battery_model.drain(self.accelerations, dt)
        self.battery -= drain
        self.battery = np.clip(self.battery, 0.0, 100.0)

        # Low battery → failsafe
        low_battery = (self.battery < 5.0) & (
            np.isin(self.status, [DroneStatus.FLYING, DroneStatus.TAKEOFF])
        )
        if np.any(low_battery):
            self.status[low_battery] = DroneStatus.FAILSAFE
            self._failsafe_count += int(np.sum(low_battery))
            logger.warning("%d drones entered FAILSAFE due to low battery.", np.sum(low_battery))

        # --- Phase 10: geofence ---
        self._check_geofence()

    # ------------------------------------------------------------------
    # Physics helpers
    # ------------------------------------------------------------------

    def _update_targets_from_show(self):
        """For each drone, interpolate position and velocity from its trajectory."""
        t = self._show_time
        for i in range(self._drone_count):
            interp = self.trajectories[i]
            if interp is not None:
                pos = interp.position_at(t)
                vel = interp.velocity_at(t)
                self.target_positions[i] = pos
                self.target_velocities[i] = vel

    def _compute_pd_acceleration(self) -> np.ndarray:
        """
        Vectorised PD controller for all drones.

        desired_acc = KP_POS * (target_pos - pos) + KD_VEL * (target_vel - vel)
        """
        pos_error = self.target_positions - self.positions      # (N, 3)
        vel_error = self.target_velocities - self.velocities    # (N, 3)
        acc = KP_POS * pos_error + KD_VEL * vel_error          # (N, 3)
        return acc

    def _update_orientations(self):
        """
        Update roll, pitch, yaw from velocity direction.

        Drones lean into their horizontal velocity (pitch forward/back,
        roll sideways).  Yaw tracks the horizontal velocity heading.
        """
        vx = self.velocities[:, 0]
        vy = self.velocities[:, 1]
        vz = self.velocities[:, 2]

        horiz_speed = np.sqrt(vx ** 2 + vy ** 2)
        total_speed = np.linalg.norm(self.velocities, axis=1)

        # Pitch: positive = nose down (forward lean)
        # Proportional to forward acceleration, capped at ±35°
        pitch = np.arctan2(vz, horiz_speed + 1e-12)  # climb/descent attitude
        pitch = np.clip(pitch, -np.radians(35.0), np.radians(35.0))

        # Roll: bank into turns — proportional to lateral acceleration
        roll = np.arctan2(self.accelerations[:, 1], 9.81 + 1e-12)
        roll = np.clip(roll, -np.radians(35.0), np.radians(35.0))

        # Yaw: heading derived from horizontal velocity
        yaw = np.arctan2(vy, vx + 1e-12)

        self.orientations[:, 0] = roll
        self.orientations[:, 1] = pitch
        self.orientations[:, 2] = yaw

    def _update_colors(self):
        """Sample LED colours from light interpolators for the current show time."""
        t = self._show_time
        for i in range(self._drone_count):
            li = self.light_interpolators[i]
            if li is not None:
                r, g, b = li.color_at(t)
                self.colors[i] = [r, g, b]

    def _check_geofence(self):
        """Set drones to FAILSAFE if they leave the geofenced volume."""
        x = self.positions[:, 0]
        y = self.positions[:, 1]
        z = self.positions[:, 2]

        horiz_dist = np.sqrt(x ** 2 + y ** 2)
        violation = (
            (horiz_dist > GEOFENCE_RADIUS)
            | (z > GEOFENCE_MAX_ALT)
            | (z < GEOFENCE_MIN_ALT)
        )
        # Only trigger for active drones (not already grounded / failsafe)
        active = np.isin(self.status, [DroneStatus.FLYING, DroneStatus.TAKEOFF, DroneStatus.ARMING])
        new_failsafe = violation & active
        if np.any(new_failsafe):
            self.status[new_failsafe] = DroneStatus.FAILSAFE
            self._failsafe_count += int(np.sum(new_failsafe))
            # Immediately zero horizontal velocity — climb toward safe alt
            self.velocities[new_failsafe, 0] = 0.0
            self.velocities[new_failsafe, 1] = 0.0
            logger.warning(
                "Geofence violation: %d drones set to FAILSAFE.", int(np.sum(new_failsafe))
            )

    def _handle_takeoff(self):
        """
        ARMING → TAKEOFF → FLYING state transitions.

        - ARMING: accumulate elapsed time; when >= ARMING_DELAY_S transition to TAKEOFF.
        - TAKEOFF: command upward movement toward HOVER_ALT.
        - When z >= HOVER_ALT - 0.5: transition to FLYING.
        """
        dt = SIM_DT

        # ---- ARMING ----
        arming_mask = self.status == DroneStatus.ARMING
        if np.any(arming_mask):
            self._arming_elapsed[arming_mask] += dt
            ready = arming_mask & (self._arming_elapsed >= ARMING_DELAY_S)
            if np.any(ready):
                self.status[ready] = DroneStatus.TAKEOFF
                # Set target to hover altitude directly above current position
                self.target_positions[ready, 0] = self.positions[ready, 0]
                self.target_positions[ready, 1] = self.positions[ready, 1]
                self.target_positions[ready, 2] = HOVER_ALT
                self.target_velocities[ready] = 0.0

        # ---- TAKEOFF ----
        takeoff_mask = self.status == DroneStatus.TAKEOFF
        if np.any(takeoff_mask):
            # Override vertical target velocity to a steady climb speed
            self.target_velocities[takeoff_mask, 2] = TAKEOFF_SPEED
            # When close enough to hover altitude, transition to FLYING
            near_hover = takeoff_mask & (self.positions[:, 2] >= HOVER_ALT - 0.5)
            if np.any(near_hover):
                self.status[near_hover] = DroneStatus.FLYING
                self.target_velocities[near_hover, 2] = 0.0
                self.target_positions[near_hover, 2] = HOVER_ALT

    def _handle_landing(self):
        """
        LANDING → GROUNDED state transitions.

        Drones in LANDING are commanded toward z = 0 at LANDING_SPEED.
        When z <= 0.1 m they are set to GROUNDED and velocities zeroed.
        """
        landing_mask = self.status == DroneStatus.LANDING
        if not np.any(landing_mask):
            return

        # Override target to descend
        self.target_positions[landing_mask, 2] = 0.0
        self.target_velocities[landing_mask, 2] = -LANDING_SPEED

        # Transition to GROUNDED
        grounded_now = landing_mask & (self.positions[:, 2] <= 0.1)
        if np.any(grounded_now):
            self.status[grounded_now] = DroneStatus.GROUNDED
            self.positions[grounded_now, 2] = 0.0
            self.velocities[grounded_now] = 0.0
            self.target_velocities[grounded_now] = 0.0

    # ------------------------------------------------------------------
    # Command API
    # ------------------------------------------------------------------

    def arm_all(self):
        """Set all IDLE or GROUNDED drones to ARMING."""
        armable = np.isin(self.status, [DroneStatus.IDLE, DroneStatus.GROUNDED])
        self.status[armable] = DroneStatus.ARMING
        self._arming_elapsed[armable] = 0.0
        logger.info("Arming %d drones.", int(np.sum(armable)))

    def start_show(self) -> bool:
        """
        Begin show execution.

        Returns False if the show is not loaded or drones are not ready.
        """
        if not self._show_loaded:
            logger.warning("start_show called but no show is loaded.")
            return False

        # Check all drones are FLYING
        not_ready = ~np.all(self.status == DroneStatus.FLYING)
        if not_ready:
            ready_count = int(np.sum(self.status == DroneStatus.FLYING))
            logger.warning(
                "start_show: only %d / %d drones are FLYING.",
                ready_count, self._drone_count
            )
            return False

        self._show_time = 0.0
        self._show_playing = True
        logger.info("Show started.")
        return True

    def stop_show(self):
        """Pause show execution; drones hover in place."""
        self._show_playing = False
        # Freeze targets at current positions
        self.target_positions[:] = self.positions
        self.target_velocities[:] = 0.0
        logger.info("Show stopped; drones hovering.")

    def land_all(self):
        """Command all flying (or hovering) drones to land."""
        self._show_playing = False
        landable = np.isin(self.status, [DroneStatus.FLYING, DroneStatus.TAKEOFF, DroneStatus.FAILSAFE])
        self.status[landable] = DroneStatus.LANDING
        # Set XY targets to current position (descend in place)
        self.target_positions[landable, 0] = self.positions[landable, 0]
        self.target_positions[landable, 1] = self.positions[landable, 1]
        logger.info("Land command issued to %d drones.", int(np.sum(landable)))

    def trigger_failsafe(self, drone_id: int, reason: str):
        """
        Set a specific drone to FAILSAFE and command return-to-home.

        The drone will hover in place (its current position becomes the target)
        and begin descending on the next land_all() call.
        """
        if drone_id < 0 or drone_id >= self._drone_count:
            raise ValueError(f"drone_id {drone_id} out of range [0, {self._drone_count})")

        self.status[drone_id] = DroneStatus.FAILSAFE
        # RTH: target is home position (above ground at x,y of initial grid position)
        self.target_positions[drone_id, 2] = HOVER_ALT
        self.target_velocities[drone_id] = 0.0
        self._failsafe_count += 1
        logger.warning("Drone %d entered FAILSAFE: %s", drone_id, reason)

    # ------------------------------------------------------------------
    # Wind configuration
    # ------------------------------------------------------------------

    def set_wind(self, speed: float, direction: float, turbulence: float = 0.0):
        """Update wind model parameters at runtime."""
        self._wind_model = WindModel(speed=speed, direction=direction, turbulence=turbulence)

    # ------------------------------------------------------------------
    # Telemetry & statistics
    # ------------------------------------------------------------------
    # Public read-only properties
    # ------------------------------------------------------------------

    @property
    def show_loaded(self) -> bool:
        return self._show_loaded

    @property
    def show_time(self) -> float:
        return self._show_time

    @property
    def show_duration(self) -> float:
        return self._show_duration

    # ------------------------------------------------------------------

    def get_telemetry(self) -> List[Dict]:
        """
        Return the current state of all drones as a list of dicts.

        Converts numpy scalars to plain Python types for JSON serialisation.
        """
        result = []
        for i in range(self._drone_count):
            result.append(
                {
                    "id": i,
                    "x": float(self.positions[i, 0]),
                    "y": float(self.positions[i, 1]),
                    "z": float(self.positions[i, 2]),
                    "vx": float(self.velocities[i, 0]),
                    "vy": float(self.velocities[i, 1]),
                    "vz": float(self.velocities[i, 2]),
                    "roll": float(self.orientations[i, 0]),
                    "pitch": float(self.orientations[i, 1]),
                    "yaw": float(self.orientations[i, 2]),
                    "r": int(self.colors[i, 0]),
                    "g": int(self.colors[i, 1]),
                    "b": int(self.colors[i, 2]),
                    "battery": float(self.battery[i]),
                    "status": int(self.status[i]),
                    "rtk_fix": bool(self.rtk_fix[i]),
                }
            )
        return result

    def get_drone_state(self, drone_id: int) -> DroneState:
        """Return a DroneState dataclass for a single drone."""
        i = drone_id
        return DroneState(
            id=i,
            x=float(self.positions[i, 0]),
            y=float(self.positions[i, 1]),
            z=float(self.positions[i, 2]),
            vx=float(self.velocities[i, 0]),
            vy=float(self.velocities[i, 1]),
            vz=float(self.velocities[i, 2]),
            roll=float(self.orientations[i, 0]),
            pitch=float(self.orientations[i, 1]),
            yaw=float(self.orientations[i, 2]),
            r=int(self.colors[i, 0]),
            g=int(self.colors[i, 1]),
            b=int(self.colors[i, 2]),
            battery=float(self.battery[i]),
            status=DroneStatus(int(self.status[i])),
            rtk_fix=bool(self.rtk_fix[i]),
        )

    def get_simulator_state(self) -> SimulatorState:
        """Return a high-level SimulatorState summary."""
        active = int(np.sum(
            np.isin(self.status, [DroneStatus.FLYING, DroneStatus.TAKEOFF, DroneStatus.ARMING])
        ))
        failsafe = int(np.sum(self.status == DroneStatus.FAILSAFE))

        return SimulatorState(
            running=self._running,
            show_loaded=self._show_loaded,
            show_time=float(self._show_time),
            show_duration=float(self._show_duration),
            drone_count=self._drone_count,
            active_drones=active,
            failsafe_drones=failsafe,
            sim_hz=float(self._sim_hz_actual),
            real_time_factor=float(self._real_time_factor),
        )

    def get_statistics(self) -> Dict:
        """Return performance and safety statistics."""
        # Compute minimum pairwise separation (sampled — not full O(N²))
        min_sep = self._compute_min_separation_sample()

        return {
            "step_count": self._step_count,
            "sim_hz": float(self._sim_hz_actual),
            "real_time_factor": float(self._real_time_factor),
            "collision_count": self._collision_count,
            "failsafe_count": self._failsafe_count,
            "min_separation_m": min_sep,
            "active_drones": int(np.sum(
                np.isin(self.status, [DroneStatus.FLYING, DroneStatus.TAKEOFF])
            )),
            "mean_battery_pct": float(np.mean(self.battery)),
            "low_battery_count": int(np.sum(self.battery < 20.0)),
        }

    def _compute_min_separation_sample(self, max_pairs: int = 500) -> float:
        """
        Estimate minimum drone separation by sampling random pairs.
        Full O(N²) is expensive for large swarms.
        """
        n = self._drone_count
        if n < 2:
            return float("inf")

        flying = np.where(
            np.isin(self.status, [DroneStatus.FLYING, DroneStatus.TAKEOFF])
        )[0]
        if len(flying) < 2:
            return float("inf")

        rng = np.random.default_rng()
        sample_size = min(max_pairs, len(flying) * (len(flying) - 1) // 2)
        min_d = float("inf")

        for _ in range(sample_size):
            i, j = rng.choice(len(flying), size=2, replace=False)
            d = float(np.linalg.norm(
                self.positions[flying[i]] - self.positions[flying[j]]
            ))
            if d < min_d:
                min_d = d
                # Track collision events
                if d < MIN_DRONE_SEPARATION:
                    self._collision_count += 1

        return min_d if min_d < float("inf") else 0.0
