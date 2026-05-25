"""
drone_math.py - Mathematical utilities for the NextGen Swarm Simulator.

Provides:
  - TrajectoryInterpolator: Catmull-Rom spline position/velocity evaluation
  - OrcaCollisionAvoider: Full ORCA (Optimal Reciprocal Collision Avoidance)
  - LBAPAssigner: Linear Bottleneck Assignment Problem solver
  - BatteryModel: Electrical drain estimation
  - WindModel: Dryden turbulence model for wind perturbations
"""

from __future__ import annotations

import math
import random
from typing import List, Tuple, Dict, Optional

import numpy as np
from scipy.optimize import linear_sum_assignment


# ---------------------------------------------------------------------------
# TrajectoryInterpolator
# ---------------------------------------------------------------------------

class TrajectoryInterpolator:
    """
    Catmull-Rom spline interpolator over a list of timed 3-D waypoints.

    Each waypoint dict must contain keys: 't', 'x', 'y', 'z'.
    The trajectory is padded at both ends so every segment always has four
    control points available.
    """

    def __init__(self, waypoints: List[Dict]):
        if len(waypoints) < 2:
            raise ValueError("TrajectoryInterpolator requires at least 2 waypoints.")

        # Sort by time
        wps = sorted(waypoints, key=lambda w: w["t"])

        self._times: np.ndarray = np.array([w["t"] for w in wps], dtype=np.float64)
        self._pts: np.ndarray = np.array(
            [[w["x"], w["y"], w["z"]] for w in wps], dtype=np.float64
        )

        # Edge-pad: duplicate first and last points so all segments have 4 CPs
        self._times_padded = np.concatenate(
            [[self._times[0]], self._times, [self._times[-1]]]
        )
        self._pts_padded = np.vstack(
            [self._pts[[0]], self._pts, self._pts[[-1]]]
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def duration(self) -> float:
        """Total show duration in seconds."""
        return float(self._times[-1] - self._times[0])

    def position_at(self, t: float) -> Tuple[float, float, float]:
        """
        Evaluate position on the Catmull-Rom spline at time *t*.

        Uses the standard centripetal Catmull-Rom formula:
            q(u) = 0.5 * ( 2*p1
                         + (-p0 + p2)*u
                         + (2*p0 - 5*p1 + 4*p2 - p3)*u^2
                         + (-p0 + 3*p1 - 3*p2 + p3)*u^3 )
        """
        t = float(np.clip(t, self._times[0], self._times[-1]))
        seg, u = self._segment_and_u(t)
        # Padded indices: segment i in original → indices i, i+1, i+2, i+3 in padded
        p0, p1, p2, p3 = (
            self._pts_padded[seg],
            self._pts_padded[seg + 1],
            self._pts_padded[seg + 2],
            self._pts_padded[seg + 3],
        )
        pos = _catmull_rom(p0, p1, p2, p3, u)
        return (float(pos[0]), float(pos[1]), float(pos[2]))

    def velocity_at(self, t: float, eps: float = 1e-4) -> Tuple[float, float, float]:
        """
        Velocity via central finite difference of the spline.
        At the boundaries a one-sided difference is used.
        """
        t0, t1 = float(self._times[0]), float(self._times[-1])
        if t - eps < t0:
            p_fwd = np.array(self.position_at(min(t + eps, t1)))
            p_bwd = np.array(self.position_at(t))
            dt = min(eps, t1 - t)
        elif t + eps > t1:
            p_fwd = np.array(self.position_at(t))
            p_bwd = np.array(self.position_at(max(t - eps, t0)))
            dt = min(eps, t - t0)
        else:
            p_fwd = np.array(self.position_at(t + eps))
            p_bwd = np.array(self.position_at(t - eps))
            dt = 2.0 * eps

        if dt < 1e-12:
            return (0.0, 0.0, 0.0)
        vel = (p_fwd - p_bwd) / dt
        return (float(vel[0]), float(vel[1]), float(vel[2]))

    def sample_positions(self, dt: float) -> np.ndarray:
        """
        Return an array of positions sampled every *dt* seconds.
        Shape: (M, 3) where M = ceil(duration / dt) + 1.
        """
        t0, t1 = float(self._times[0]), float(self._times[-1])
        ts = np.arange(t0, t1 + dt, dt)
        out = np.empty((len(ts), 3), dtype=np.float64)
        for i, t in enumerate(ts):
            out[i] = self.position_at(t)
        return out

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _segment_and_u(self, t: float) -> Tuple[int, float]:
        """
        Return (segment_index, u) where segment_index indexes into
        *self._times* (original, not padded) and u ∈ [0, 1] is the
        normalised parameter within that segment.
        """
        idx = int(np.searchsorted(self._times, t, side="right")) - 1
        idx = int(np.clip(idx, 0, len(self._times) - 2))

        t_lo = self._times[idx]
        t_hi = self._times[idx + 1]
        span = t_hi - t_lo
        u = float((t - t_lo) / span) if span > 1e-12 else 0.0
        u = float(np.clip(u, 0.0, 1.0))
        return idx, u


def _catmull_rom(
    p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, u: float
) -> np.ndarray:
    """Standard centripetal Catmull-Rom formula for a single parameter u."""
    u2 = u * u
    u3 = u2 * u
    return 0.5 * (
        2.0 * p1
        + (-p0 + p2) * u
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3
    )


# ---------------------------------------------------------------------------
# OrcaCollisionAvoider
# ---------------------------------------------------------------------------

class OrcaCollisionAvoider:
    """
    Full ORCA (Optimal Reciprocal Collision Avoidance) for N drones.

    Each drone computes half-plane constraints from neighbours and solves a
    small 2-D (or 3-D) LP to find the closest realisable velocity to its
    preferred velocity.

    Reference: van den Berg et al., "Reciprocal n-Body Collision Avoidance",
    ISRR 2009.
    """

    def __init__(
        self,
        min_distance: float = 2.0,
        time_horizon: float = 3.0,
        max_speed: float = 8.0,
    ):
        self.min_distance = min_distance
        self.time_horizon = time_horizon
        self.max_speed = max_speed

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_new_velocities(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        preferred_velocities: np.ndarray,
    ) -> np.ndarray:
        """
        Compute ORCA-adjusted velocities for all N drones.

        Parameters
        ----------
        positions          : (N, 3) float64 – current positions
        velocities         : (N, 3) float64 – current velocities
        preferred_velocities: (N, 3) float64 – unconstrained desired velocities

        Returns
        -------
        new_velocities : (N, 3) float64
        """
        N = len(positions)
        new_velocities = np.copy(preferred_velocities)

        # Build spatial grid for O(N) neighbour lookup
        grid = self._build_spatial_grid(positions, cell_size=max(5.0, self.min_distance * 2.5))

        combined_r = self.min_distance  # each drone contributes half

        for i in range(N):
            pos_i = positions[i]
            vel_i = velocities[i]
            pref_i = preferred_velocities[i]

            neighbours = self._get_neighbours(i, pos_i, positions, grid)

            halfplanes: List[Tuple[np.ndarray, np.ndarray]] = []
            for j in neighbours:
                pos_j = positions[j]
                vel_j = velocities[j]
                diff = pos_j - pos_i
                dist = float(np.linalg.norm(diff))
                if dist > (combined_r + self.max_speed * self.time_horizon) * 1.5:
                    continue  # too far to matter

                normal, point = self._compute_vo_halfplane(
                    pos_i, pos_j, vel_i, vel_j, combined_r, self.time_horizon
                )
                halfplanes.append((normal, point))

            if halfplanes:
                new_velocities[i] = self._solve_orca_lp(pref_i, halfplanes, self.max_speed)

        return new_velocities

    # ------------------------------------------------------------------
    # Spatial grid
    # ------------------------------------------------------------------

    def _build_spatial_grid(
        self, positions: np.ndarray, cell_size: float = 5.0
    ) -> Dict[Tuple[int, int, int], List[int]]:
        """
        Build a 3-D spatial hash grid.  Returns dict: cell_key -> [drone indices].
        """
        grid: Dict[Tuple[int, int, int], List[int]] = {}
        inv = 1.0 / cell_size
        for idx, pos in enumerate(positions):
            key = (int(math.floor(pos[0] * inv)),
                   int(math.floor(pos[1] * inv)),
                   int(math.floor(pos[2] * inv)))
            grid.setdefault(key, []).append(idx)
        return grid

    def _get_neighbours(
        self,
        i: int,
        pos_i: np.ndarray,
        positions: np.ndarray,
        grid: Dict,
    ) -> List[int]:
        """Return indices of drones that could potentially conflict with drone i."""
        cell_size = max(5.0, self.min_distance * 2.5)
        inv = 1.0 / cell_size
        cx = int(math.floor(pos_i[0] * inv))
        cy = int(math.floor(pos_i[1] * inv))
        cz = int(math.floor(pos_i[2] * inv))

        neighbours: List[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    key = (cx + dx, cy + dy, cz + dz)
                    for j in grid.get(key, []):
                        if j != i:
                            neighbours.append(j)
        return neighbours

    # ------------------------------------------------------------------
    # VO half-plane computation
    # ------------------------------------------------------------------

    def _compute_vo_halfplane(
        self,
        pos_i: np.ndarray,
        pos_j: np.ndarray,
        vel_i: np.ndarray,
        vel_j: np.ndarray,
        combined_r: float,
        tau: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the ORCA half-plane for drone i due to drone j.

        The ORCA constraint is:
            (new_vel_i - u_i_opt) · n >= 0
        where n points away from the VO cone and u_i_opt is the point on
        the VO boundary closest to the relative velocity.

        Returns (normal, point) — the half-plane is {v : (v - point)·normal >= 0}.
        """
        # Relative position and velocity
        rel_pos = pos_j - pos_i          # vector from i to j
        rel_vel = vel_i - vel_j          # relative velocity of i w.r.t. j

        dist_sq = float(np.dot(rel_pos, rel_pos))
        dist = math.sqrt(dist_sq) if dist_sq > 1e-12 else 1e-6

        # Scaled by 1/tau: the truncated VO cone
        r_tau = combined_r / tau
        w = rel_vel - rel_pos / tau     # vector from VO apex to relative velocity

        w_len_sq = float(np.dot(w, w))

        # Check if relative velocity is inside the VO sphere (already colliding)
        if w_len_sq < r_tau * r_tau:
            # Already inside: push out along w direction
            w_len = math.sqrt(w_len_sq) if w_len_sq > 1e-12 else 1e-6
            normal = w / w_len           # point outward from sphere
            # u is the displacement needed to reach the boundary
            u = (r_tau - w_len) * normal
            # Half-plane point: current rel_vel + half the correction
            point = vel_i + 0.5 * u
            return normal, point

        # Check which part of VO boundary (sphere vs cone sides) is closest
        dot_w_relpos = float(np.dot(w, rel_pos / tau))
        leg = math.sqrt(max(0.0, dist_sq / (tau * tau) - r_tau * r_tau))

        if dot_w_relpos < 0.0 and dot_w_relpos * dot_w_relpos > w_len_sq * (r_tau * r_tau / (dist_sq / (tau * tau))):
            # Closest to the circular cap
            w_len = math.sqrt(w_len_sq) if w_len_sq > 1e-12 else 1e-6
            normal = w / w_len
            u = (r_tau - w_len) * normal
        else:
            # Closest to the cone side
            # Left or right leg depending on cross product sign
            rel_pos_unit = rel_pos / (dist + 1e-12)
            # Choose the leg direction
            cross = np.cross(rel_pos_unit, w)
            if np.dot(cross, cross) < 1e-12:
                # Degenerate — use simple outward direction
                normal = -rel_pos_unit
                u = (r_tau - math.sqrt(w_len_sq)) * normal if w_len_sq > 0 else normal
            else:
                cross_len = float(np.linalg.norm(cross))
                sin_theta = float(np.clip(r_tau * tau / dist, -1.0, 1.0))
                cos_theta = math.sqrt(max(0.0, 1.0 - sin_theta * sin_theta))

                # Rotate rel_pos_unit by ±theta around the cross axis
                sign = 1.0 if float(np.dot(cross, np.array([0.0, 0.0, 1.0]))) >= 0.0 else -1.0
                # Build leg direction in the plane of rel_pos and up
                perp = np.cross(rel_pos_unit, cross / (cross_len + 1e-12))
                leg_dir = cos_theta * rel_pos_unit + sign * sin_theta * perp
                leg_dir_norm = float(np.linalg.norm(leg_dir))
                if leg_dir_norm > 1e-12:
                    leg_dir /= leg_dir_norm
                else:
                    leg_dir = -rel_pos_unit

                # Normal is perpendicular to the leg direction (outward)
                normal = np.array([-leg_dir[1], leg_dir[0], 0.0])
                normal_len = float(np.linalg.norm(normal))
                if normal_len > 1e-12:
                    normal /= normal_len
                else:
                    normal = np.array([0.0, 0.0, 1.0])

                proj = float(np.dot(w, normal))
                u = (abs(proj) if proj < 0 else 0.0) * normal

        point = vel_i + 0.5 * u
        return normal, point

    # ------------------------------------------------------------------
    # LP solver
    # ------------------------------------------------------------------

    def _solve_orca_lp(
        self,
        preferred_vel: np.ndarray,
        halfplanes: List[Tuple[np.ndarray, np.ndarray]],
        max_speed: float,
    ) -> np.ndarray:
        """
        Find the velocity closest to *preferred_vel* that satisfies all
        half-plane constraints and lies within the max-speed sphere.

        Uses an incremental LP approach: start with the preferred velocity,
        project it to each half-plane in turn if violated, and fall back to
        the feasibility LP if a conflict is detected.
        """
        vel = np.copy(preferred_vel)

        # Clamp to max speed sphere first
        spd = float(np.linalg.norm(vel))
        if spd > max_speed:
            vel = vel * (max_speed / spd)

        for k, (normal, point) in enumerate(halfplanes):
            # Check if current velocity satisfies this half-plane
            # Constraint: (vel - point) · normal >= 0
            if float(np.dot(vel - point, normal)) >= -1e-8:
                continue  # already satisfied

            # Project vel onto the half-plane boundary line (3D: project onto
            # the plane defined by normal, then pick closest feasible point)
            # The boundary is the set {v : (v - point)·normal = 0}
            # Projection: vel_proj = vel - ((vel-point)·n) * n
            proj = vel - float(np.dot(vel - point, normal)) * normal

            # Check if projected velocity is within max speed
            proj_spd = float(np.linalg.norm(proj))
            if proj_spd > max_speed:
                # Intersection of half-plane boundary with max-speed sphere
                # Find the closest point on the boundary line to origin within the sphere
                # Boundary line: point + t * tangent  (tangent perp to normal)
                # Simplification: scale to max_speed
                proj = proj * (max_speed / proj_spd)

            # Check if this satisfies all previous half-planes
            feasible = True
            for (n2, p2) in halfplanes[:k]:
                if float(np.dot(proj - p2, n2)) < -1e-6:
                    feasible = False
                    break

            if feasible:
                vel = proj
            else:
                # Fall back: try to find a feasible velocity by minimally
                # violating constraints — use the point on the current
                # constraint boundary closest to preferred velocity
                best = None
                best_dist = float("inf")

                # Sample a few candidate points on this half-plane boundary
                tangent_candidates = self._halfplane_boundary_candidates(
                    normal, point, preferred_vel, max_speed, n_samples=16
                )
                for cand in tangent_candidates:
                    ok = True
                    for (n2, p2) in halfplanes[:k]:
                        if float(np.dot(cand - p2, n2)) < -1e-6:
                            ok = False
                            break
                    if ok:
                        d = float(np.linalg.norm(cand - preferred_vel))
                        if d < best_dist:
                            best_dist = d
                            best = cand

                if best is not None:
                    vel = best
                # If no feasible point found, keep current vel (best effort)

        return vel

    def _halfplane_boundary_candidates(
        self,
        normal: np.ndarray,
        point: np.ndarray,
        preferred: np.ndarray,
        max_speed: float,
        n_samples: int = 16,
    ) -> List[np.ndarray]:
        """
        Generate candidate points on the half-plane boundary
        {v : (v-point)·normal = 0} within the speed sphere.
        """
        # Build two orthogonal tangent vectors
        t1 = _perp_vector(normal)
        t2 = np.cross(normal, t1)
        t2_len = float(np.linalg.norm(t2))
        if t2_len > 1e-12:
            t2 /= t2_len

        # Project preferred onto the boundary plane
        proj_pref = preferred - float(np.dot(preferred - point, normal)) * normal

        candidates: List[np.ndarray] = []
        # The boundary plane intersects the speed sphere in a circle;
        # parameterise it
        for i in range(n_samples):
            angle = 2.0 * math.pi * i / n_samples
            cand = proj_pref + max_speed * 0.1 * (math.cos(angle) * t1 + math.sin(angle) * t2)
            # Clamp to sphere
            spd = float(np.linalg.norm(cand))
            if spd > max_speed:
                cand = cand * (max_speed / spd)
            candidates.append(cand)

        # Also include the raw projection
        spd = float(np.linalg.norm(proj_pref))
        if spd > max_speed:
            proj_pref = proj_pref * (max_speed / spd)
        candidates.append(proj_pref)
        return candidates


def _perp_vector(v: np.ndarray) -> np.ndarray:
    """Return an arbitrary unit vector perpendicular to v."""
    v = v / (np.linalg.norm(v) + 1e-12)
    if abs(v[0]) < 0.9:
        w = np.array([1.0, 0.0, 0.0])
    else:
        w = np.array([0.0, 1.0, 0.0])
    p = w - float(np.dot(w, v)) * v
    p_len = float(np.linalg.norm(p))
    if p_len > 1e-12:
        return p / p_len
    return np.array([0.0, 0.0, 1.0])


# ---------------------------------------------------------------------------
# LBAPAssigner
# ---------------------------------------------------------------------------

class LBAPAssigner:
    """
    Assigns N drones to N show slots.

    Two methods:
      - 'lbap':     minimise the maximum travel distance (bottleneck assignment).
      - 'hungarian': minimise the total travel distance (sum assignment).
    """

    def __init__(self):
        pass

    def assign(
        self,
        current_positions: np.ndarray,
        target_positions: np.ndarray,
        method: str = "lbap",
    ) -> np.ndarray:
        """
        Compute assignment.

        Returns
        -------
        assignment : (N,) int array
            assignment[i] = j  →  drone i goes to target slot j.
        """
        cost_matrix = self._compute_cost_matrix(current_positions, target_positions)

        if method == "lbap":
            row_ind, col_ind = self._lbap_binary_search(cost_matrix)
        elif method == "hungarian":
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
        else:
            raise ValueError(f"Unknown assignment method: {method!r}")

        N = len(current_positions)
        assignment = np.empty(N, dtype=np.int64)
        assignment[row_ind] = col_ind
        return assignment

    def _compute_cost_matrix(
        self, positions_a: np.ndarray, positions_b: np.ndarray
    ) -> np.ndarray:
        """
        Euclidean distance matrix, shape (N, N).
        Uses broadcasting for efficiency.
        """
        # positions_a: (N, 3),  positions_b: (M, 3)
        # out[i, j] = ||a[i] - b[j]||
        diff = positions_a[:, np.newaxis, :] - positions_b[np.newaxis, :, :]
        return np.sqrt(np.sum(diff * diff, axis=-1))

    def _lbap_binary_search(
        self, cost_matrix: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Binary search over sorted unique costs to find the minimum bottleneck
        threshold for which a perfect matching exists.

        Algorithm
        ---------
        1. Collect and sort unique cost values.
        2. Binary search: at each midpoint threshold t,
           build adjacency matrix adj[i,j] = 1 if cost[i,j] <= t.
        3. Run Hungarian on -adj.  If all matched weights are 1 (perfect
           matching), the threshold is feasible → try lower.
        4. Return the best (lowest threshold) assignment found.
        """
        flat_costs = np.unique(cost_matrix.ravel())
        N = cost_matrix.shape[0]

        lo, hi = 0, len(flat_costs) - 1
        best_row = np.arange(N, dtype=np.int64)
        best_col = np.arange(N, dtype=np.int64)

        while lo <= hi:
            mid = (lo + hi) // 2
            threshold = flat_costs[mid]

            adj = (cost_matrix <= threshold).astype(np.float64)
            row_ind, col_ind = linear_sum_assignment(-adj)

            # Check if perfect matching: every assigned cell has adj value 1
            matched_weights = adj[row_ind, col_ind]
            if np.all(matched_weights >= 1.0 - 1e-9) and len(row_ind) == N:
                best_row = row_ind
                best_col = col_ind
                hi = mid - 1  # try to improve (lower threshold)
            else:
                lo = mid + 1  # need higher threshold

        return best_row, best_col

    def assignment_quality(
        self, assignment: np.ndarray, cost_matrix: np.ndarray
    ) -> Dict:
        """
        Quality metrics for a given assignment.

        Returns dict with keys:
          max_dist, min_dist, mean_dist, total_dist, fairness_ratio.

        fairness_ratio = min_dist / max_dist  (1.0 = perfectly fair).
        """
        N = len(assignment)
        distances = np.array(
            [cost_matrix[i, assignment[i]] for i in range(N)], dtype=np.float64
        )
        max_d = float(np.max(distances))
        min_d = float(np.min(distances))
        mean_d = float(np.mean(distances))
        total_d = float(np.sum(distances))
        fairness = (min_d / max_d) if max_d > 1e-12 else 1.0

        return {
            "max_dist": max_d,
            "min_dist": min_d,
            "mean_dist": mean_d,
            "total_dist": total_d,
            "fairness_ratio": fairness,
        }


# ---------------------------------------------------------------------------
# BatteryModel
# ---------------------------------------------------------------------------

class BatteryModel:
    """
    Simple Peukert-inspired battery drain model for a 6S LiPo.

    Hover current is the baseline; maneuvering (acceleration) costs extra
    proportional to |a|.
    """

    def __init__(
        self,
        capacity_mah: float = 4000.0,
        voltage: float = 22.2,
        hover_current: float = 8.0,
    ):
        self.capacity_mah = capacity_mah
        self.voltage = voltage
        self.hover_current = hover_current

        # Capacity in mAs
        self._capacity_mas = capacity_mah * 3600.0  # mA·s

        # Empirical: each 1 m/s² of acceleration adds this many mA of extra current
        self._maneuver_current_per_acc = 2.5  # mA per (m/s²)

    def drain(self, acceleration: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute the percentage of battery drained in one timestep for each drone.

        Parameters
        ----------
        acceleration : (N, 3) float64 – drone acceleration vectors
        dt           : timestep in seconds

        Returns
        -------
        drain_pct : (N,) float64 – percentage drained [0, 100]
        """
        acc_magnitude = np.linalg.norm(acceleration, axis=1)  # (N,)
        current = self.hover_current + self._maneuver_current_per_acc * acc_magnitude  # mA
        charge_used_mas = current * dt                         # mA·s
        drain_pct = (charge_used_mas / self._capacity_mas) * 100.0
        return drain_pct

    def estimate_remaining_time(self, battery_pct: np.ndarray) -> np.ndarray:
        """
        Estimate remaining hover time in minutes for each drone.

        Parameters
        ----------
        battery_pct : (N,) float64 – current battery levels [0, 100]

        Returns
        -------
        remaining_minutes : (N,) float64
        """
        # Charge remaining in mAs
        remaining_mas = (battery_pct / 100.0) * self._capacity_mas
        # Time at hover current (worst-case estimate)
        remaining_s = remaining_mas / (self.hover_current * 1000.0)  # hover_current in A → mA×1000
        # Correct: hover_current is already in A? Let's keep units consistent.
        # capacity_mah * 3600 = capacity_mas  (mA·s)
        # current is in A → need to convert: current_ma = hover_current_A * 1000
        # Recalculate properly:
        hover_current_ma = self.hover_current * 1000.0  # Amps → mA
        remaining_s = remaining_mas / hover_current_ma
        return remaining_s / 60.0  # convert to minutes


# ---------------------------------------------------------------------------
# WindModel  (Dryden turbulence model)
# ---------------------------------------------------------------------------

class WindModel:
    """
    Wind and turbulence model using a simplified Dryden spectral model.

    The Dryden model shapes white noise through first-order shaping filters
    to produce turbulence with realistic spectral content.

    Reference: MIL-HDBK-1797 / MIL-F-8785C.
    """

    def __init__(
        self,
        speed: float = 0.0,
        direction: float = 0.0,  # degrees, measured from north, clockwise
        turbulence: float = 0.0,  # turbulence intensity [0, 1]
    ):
        self.speed = speed
        self.direction = direction
        self.turbulence = turbulence

        # Mean wind vector (XY plane, Z=0)
        dir_rad = math.radians(direction)
        self._mean_wind = np.array(
            [speed * math.sin(dir_rad), speed * math.cos(dir_rad), 0.0],
            dtype=np.float64,
        )

        # Dryden turbulence parameters
        # Length scales (m) — typical low-altitude values
        self._L_u = 200.0   # longitudinal
        self._L_v = 200.0
        self._L_w = 50.0    # vertical (shorter scale near ground)

        # Turbulence intensities (m/s RMS)
        base_sigma = turbulence * max(1.0, speed * 0.15)
        self._sigma_u = base_sigma
        self._sigma_v = base_sigma
        self._sigma_w = base_sigma * 0.6  # vertical is weaker

        # Internal filter state for Dryden (first-order Gauss-Markov)
        # Shape: we'll maintain per-call state lazily
        self._state_u: Optional[float] = None
        self._state_v: Optional[float] = None
        self._state_w: Optional[float] = None
        self._last_t: float = -1.0
        self._dt: float = 0.01  # expected dt

    def get_force(self, positions: np.ndarray, t: float) -> np.ndarray:
        """
        Compute wind force vectors for N drones.

        The base force is the mean wind.  Turbulent perturbations are added
        using a Dryden-shaped noise process (spatially uniform for simplicity,
        sufficient for flight dynamics testing).

        Parameters
        ----------
        positions : (N, 3) float64
        t         : current simulation time (seconds)

        Returns
        -------
        forces : (N, 3) float64 – wind force perturbation vectors (m/s equivalent)
        """
        N = len(positions)
        dt = t - self._last_t if self._last_t >= 0.0 else self._dt
        dt = max(1e-6, min(dt, 1.0))
        self._last_t = t

        if self.turbulence < 1e-6 and self.speed < 1e-6:
            return np.zeros((N, 3), dtype=np.float64)

        # --- Dryden turbulence via first-order shaping filter ---
        # Continuous first-order model:  dx/dt = -x/tau + sigma*sqrt(2/tau)*w(t)
        # Discretised: x[k+1] = x[k]*exp(-dt/tau) + sigma*sqrt(1-exp(-2dt/tau))*N(0,1)

        def dryden_step(state, sigma, L, V_ref=5.0):
            """One step of the Dryden filter. V_ref is reference airspeed (m/s)."""
            if sigma < 1e-9:
                return 0.0, 0.0
            tau = L / (V_ref + 1e-6)
            alpha = math.exp(-dt / tau)
            noise_scale = sigma * math.sqrt(max(0.0, 1.0 - alpha * alpha))
            if state is None:
                state = random.gauss(0.0, sigma)
            new_state = alpha * state + noise_scale * random.gauss(0.0, 1.0)
            return new_state, new_state

        V_ref = max(1.0, self.speed)
        self._state_u, turb_u = dryden_step(self._state_u, self._sigma_u, self._L_u, V_ref)
        self._state_v, turb_v = dryden_step(self._state_v, self._sigma_v, self._L_v, V_ref)
        self._state_w, turb_w = dryden_step(self._state_w, self._sigma_w, self._L_w, V_ref)

        # Rotate turbulence into world frame (aligned with mean wind direction)
        dir_rad = math.radians(self.direction)
        cos_d = math.cos(dir_rad)
        sin_d = math.sin(dir_rad)

        # Longitudinal (u) is along wind direction, lateral (v) is 90° left
        turb_world = np.array(
            [
                turb_u * sin_d - turb_v * cos_d,  # x
                turb_u * cos_d + turb_v * sin_d,  # y
                turb_w,                             # z
            ],
            dtype=np.float64,
        )

        # Total wind = mean + turbulence, broadcast to all drones
        total_wind = self._mean_wind + turb_world
        forces = np.tile(total_wind, (N, 1))  # (N, 3)

        # Optional: vary slightly with altitude (wind shear — logarithmic profile)
        if N > 0 and self.speed > 0.5:
            z_ref = 10.0  # reference altitude
            altitudes = positions[:, 2]
            z_safe = np.maximum(altitudes, 0.5)
            shear = np.log(z_safe / 0.1 + 1.0) / (math.log(z_ref / 0.1 + 1.0) + 1e-12)
            shear = np.clip(shear, 0.0, 2.0)
            forces[:, :2] *= shear[:, np.newaxis]

        return forces
