"""
Demo show generator — creates built-in choreography sequences for testing.

Sequences (in order):
  1. Circle launch      — drones ascend from grid, form horizontal ring
  2. Sphere             — ring expands/contracts into 3D sphere
  3. Text "HI"          — flatten to vertical letter shapes
  4. Spiral             — drones spiral outward and upward
  5. Landing            — return to grid and descend
"""
from __future__ import annotations

import math
import json
import zipfile
import io
from dataclasses import dataclass, field
from typing import NamedTuple


# ---------------------------------------------------------------------------
# Minimal data types (mirror of skyc_reader to avoid circular imports)
# ---------------------------------------------------------------------------

class Waypoint(NamedTuple):
    t: float
    x: float
    y: float
    z: float


class LightKeyframe(NamedTuple):
    t: float
    r: int
    g: int
    b: int


@dataclass
class DemoTrajectory:
    waypoints: list[Waypoint] = field(default_factory=list)
    light_cues: list[LightKeyframe] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Formation helpers
# ---------------------------------------------------------------------------

def _grid_positions(n: int, spacing: float = 2.5) -> list[tuple[float, float]]:
    """Return ground-level (x, y) grid positions for n drones."""
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    cx = (cols - 1) * spacing / 2.0
    cy = (rows - 1) * spacing / 2.0
    positions: list[tuple[float, float]] = []
    for row in range(rows):
        for col in range(cols):
            if len(positions) >= n:
                break
            positions.append((col * spacing - cx, row * spacing - cy))
    return positions


def _circle_positions(n: int, radius: float, z: float) -> list[tuple[float, float, float]]:
    positions = []
    for i in range(n):
        angle = 2.0 * math.pi * i / n
        positions.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return positions


def _sphere_positions(n: int, radius: float, cx: float = 0, cy: float = 0, cz: float = 30) -> list[tuple[float, float, float]]:
    """Fibonacci sphere distribution."""
    positions = []
    golden = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(n):
        y = 1.0 - (i / float(n - 1)) * 2.0 if n > 1 else 0.0
        r = math.sqrt(max(0.0, 1.0 - y * y))
        theta = golden * i
        positions.append((
            cx + radius * r * math.cos(theta),
            cy + radius * r * math.sin(theta),
            cz + radius * y,
        ))
    return positions


def _spiral_positions(n: int, radius: float, height_step: float, turns: float = 2.0) -> list[tuple[float, float, float]]:
    positions = []
    for i in range(n):
        frac = i / max(n - 1, 1)
        angle = frac * turns * 2.0 * math.pi
        r = radius * (0.3 + 0.7 * frac)
        positions.append((
            r * math.cos(angle),
            r * math.sin(angle),
            10.0 + frac * height_step,
        ))
    return positions


def _text_hi_positions(n: int, z: float = 30.0) -> list[tuple[float, float, float]]:
    """Approximate positions for the letters 'HI' using dots."""
    # Letter H occupies x in [-14, -5], I in [-1, 6]
    # Build a simple dot grid for each letter
    h_points: list[tuple[float, float]] = []
    # Left vertical bar
    for row in range(7):
        h_points.append((-14.0, -9.0 + row * 3.0))
    # Right vertical bar
    for row in range(7):
        h_points.append((-6.0, -9.0 + row * 3.0))
    # Cross bar
    for col in range(3):
        h_points.append((-12.0 + col * 3.0, 0.0))

    i_points: list[tuple[float, float]] = []
    for row in range(7):
        i_points.append((2.0, -9.0 + row * 3.0))

    all_pts = h_points + i_points
    # Repeat or trim to exactly n
    if len(all_pts) == 0:
        all_pts = [(0.0, 0.0)]
    while len(all_pts) < n:
        all_pts = (all_pts * 2)[:n]
    all_pts = all_pts[:n]

    return [(x, 0.0, z + y) for (x, y) in all_pts]


# ---------------------------------------------------------------------------
# Colour sequences
# ---------------------------------------------------------------------------

def _hsv_to_rgb(h: float, s: float, v: float) -> tuple[int, int, int]:
    h = h % 1.0
    i = int(h * 6)
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    sectors = [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)]
    r, g, b = sectors[i % 6]
    return int(r * 255), int(g * 255), int(b * 255)


def _drone_hue(drone_idx: int, total: int) -> float:
    return drone_idx / max(total, 1)


# ---------------------------------------------------------------------------
# Main generator
# ---------------------------------------------------------------------------

def generate_demo_show(
    drone_count: int = 100,
    show_title: str = "Demo Show",
) -> dict:
    """
    Return a show dict compatible with SkycReader.load() output format:
    {
        "metadata": {...},
        "trajectories": [{"drone_id": i, "waypoints": [...]}],
        "light_cues": [{"drone_id": i, "keyframes": [...]}],
    }
    """
    n = drone_count
    grid = _grid_positions(n)
    trajectories: list[dict] = []
    light_cues: list[dict] = []

    # Timeline (seconds)
    T_GROUND   = 0.0
    T_LIFT_END = 5.0
    T_CIRCLE   = 10.0
    T_SPHERE   = 20.0
    T_HI_START = 28.0
    T_HI_END   = 38.0
    T_SPIRAL   = 46.0
    T_LAND_START = 56.0
    T_LAND_END   = 64.0
    DURATION = T_LAND_END

    circle_pts  = _circle_positions(n, radius=max(10.0, n * 0.12), z=20.0)
    sphere_pts  = _sphere_positions(n, radius=max(12.0, n * 0.10))
    hi_pts      = _text_hi_positions(n, z=30.0)
    spiral_pts  = _spiral_positions(n, radius=max(12.0, n * 0.10), height_step=25.0)

    for i in range(n):
        gx, gy = grid[i]
        cx, cy, cz = circle_pts[i]
        sx, sy, sz = sphere_pts[i]
        hx, hy, hz = hi_pts[i]
        px, py, pz = spiral_pts[i]
        hue = _drone_hue(i, n)

        wp: list[dict] = [
            # On the ground
            {"t": T_GROUND,   "x": gx,  "y": gy,  "z": 0.0},
            # Lift to circle altitude
            {"t": T_LIFT_END, "x": cx,  "y": cy,  "z": cz},
            # Hold circle
            {"t": T_CIRCLE,   "x": cx,  "y": cy,  "z": cz},
            # Expand to sphere
            {"t": T_SPHERE,   "x": sx,  "y": sy,  "z": sz},
            # Transition to HI
            {"t": T_HI_START, "x": hx,  "y": hy,  "z": hz},
            # Hold HI
            {"t": T_HI_END,   "x": hx,  "y": hy,  "z": hz},
            # Spiral outward
            {"t": T_SPIRAL,   "x": px,  "y": py,  "z": pz},
            # Return to ground
            {"t": T_LAND_START, "x": gx, "y": gy, "z": 8.0},
            {"t": T_LAND_END,   "x": gx, "y": gy, "z": 0.0},
        ]

        # Light cues — each phase has its own colour palette
        r0, g0, b0 = _hsv_to_rgb(hue, 0.3, 0.8)       # lift: soft white-ish
        r1, g1, b1 = _hsv_to_rgb(hue, 1.0, 1.0)        # circle: saturated
        r2, g2, b2 = _hsv_to_rgb(hue + 0.33, 1.0, 1.0) # sphere: shifted hue
        r3, g3, b3 = 255, 255, 255                       # HI: white
        r4, g4, b4 = _hsv_to_rgb(hue + 0.67, 1.0, 1.0) # spiral: another shift
        r5, g5, b5 = _hsv_to_rgb(hue, 0.2, 0.4)         # landing: dim

        lc: list[dict] = [
            {"t": T_GROUND,    "r": 0,   "g": 0,   "b": 0},
            {"t": T_LIFT_END,  "r": r0,  "g": g0,  "b": b0},
            {"t": T_CIRCLE,    "r": r1,  "g": g1,  "b": b1},
            {"t": T_SPHERE,    "r": r2,  "g": g2,  "b": b2},
            {"t": T_HI_START,  "r": r3,  "g": g3,  "b": b3},
            {"t": T_HI_END,    "r": r3,  "g": g3,  "b": b3},
            {"t": T_SPIRAL,    "r": r4,  "g": g4,  "b": b4},
            {"t": T_LAND_START,"r": r5,  "g": g5,  "b": b5},
            {"t": T_LAND_END,  "r": 0,   "g": 0,   "b": 0},
        ]

        trajectories.append({"drone_id": i, "waypoints": wp})
        light_cues.append({"drone_id": i, "keyframes": lc})

    return {
        "metadata": {
            "version": "1.0",
            "title": show_title,
            "duration": DURATION,
            "drone_count": n,
            "author": "NextGen Swarm Sim",
            "settings": {"min_altitude": 0, "max_altitude": 60, "geofence_radius": 80},
        },
        "trajectories": trajectories,
        "light_cues": light_cues,
    }


def build_skyc_bytes(drone_count: int = 100, title: str = "Demo Show") -> bytes:
    """Pack a demo show into a .skyc ZIP bytes object."""
    show = generate_demo_show(drone_count, title)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        meta = {k: v for k, v in show["metadata"].items()}
        zf.writestr("show.json", json.dumps(meta, indent=2))
        for traj in show["trajectories"]:
            idx = traj["drone_id"]
            zf.writestr(f"trajectories/drone_{idx:04d}.json",
                        json.dumps({"waypoints": traj["waypoints"]}, indent=2))
        for lc in show["light_cues"]:
            idx = lc["drone_id"]
            zf.writestr(f"lights/drone_{idx:04d}.json",
                        json.dumps({"keyframes": lc["keyframes"]}, indent=2))
    return buf.getvalue()


if __name__ == "__main__":
    import sys
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    data = build_skyc_bytes(n, "CLI Demo Show")
    path = f"demo_{n}_drones.skyc"
    with open(path, "wb") as f:
        f.write(data)
    print(f"Generated {path} ({len(data)} bytes, {n} drones)")
