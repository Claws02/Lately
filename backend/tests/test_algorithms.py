"""
Unit tests for the core swarm algorithms:
  - LBAP task assignment (bottleneck minimization)
  - Hungarian assignment (total distance minimization)
  - Geofencing
  - Show file parsing
  - Demo show generation
"""
import math
import sys
import os

import numpy as np
import pytest

# Ensure source is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "simulation"))


# ---------------------------------------------------------------------------
# LBAP / Hungarian
# ---------------------------------------------------------------------------

class TestTaskAssignment:
    def _get_assigner(self):
        from src.fleet_manager.task_assigner import TaskAssigner
        return TaskAssigner()

    def test_hungarian_trivial(self):
        a = TaskAssigner = self._get_assigner()
        current = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
        targets = np.array([[10.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        assignment = a.hungarian_assign(current, targets)
        # Each drone should swap
        assert len(assignment) == 2
        assert set(assignment) == {0, 1}

    def test_lbap_equalizes_max_distance(self):
        a = self._get_assigner()
        # Drone 0 at origin, Drone 1 at (5,0,0)
        # Target A at (100,0,0), Target B at (6,0,0)
        # Hungarian might send D0 -> A (dist=100) and D1 -> B (dist=1)  total=101 max=100
        # LBAP should balance: D0->B (dist=6) D1->A (dist=95)... actually let's check
        # Better test: symmetric case
        n = 4
        current = np.zeros((n, 3))
        current[:, 0] = [0, 1, 2, 3]
        targets = np.zeros((n, 3))
        targets[:, 0] = [3, 2, 1, 0]  # reverse order

        lbap_assign = a.lbap_assign(current, targets)
        h_assign = a.hungarian_assign(current, targets)

        cm = a.compute_cost_matrix(current, targets)

        lbap_max = max(cm[i, lbap_assign[i]] for i in range(n))
        h_max = max(cm[i, h_assign[i]] for i in range(n))

        # LBAP max should be <= Hungarian max
        assert lbap_max <= h_max + 1e-6

    def test_assignment_is_permutation(self):
        a = self._get_assigner()
        n = 20
        rng = np.random.default_rng(42)
        current = rng.uniform(-50, 50, (n, 3))
        targets = rng.uniform(-50, 50, (n, 3))
        for method in ("lbap", "hungarian"):
            assign = a.lbap_assign(current, targets) if method == "lbap" else a.hungarian_assign(current, targets)
            assert len(assign) == n
            assert len(set(assign)) == n  # bijection

    def test_stats_keys(self):
        a = self._get_assigner()
        n = 5
        rng = np.random.default_rng(0)
        c = rng.uniform(0, 10, (n, 3))
        t = rng.uniform(0, 10, (n, 3))
        assign = a.lbap_assign(c, t)
        cm = a.compute_cost_matrix(c, t)
        stats = a.assignment_stats(assign, cm)
        for key in ("max_cost", "min_cost", "mean_cost", "total_cost"):
            assert key in stats

    def test_large_swarm_performance(self):
        """LBAP should complete in reasonable time for 200 drones."""
        import time
        a = self._get_assigner()
        n = 200
        rng = np.random.default_rng(7)
        current = rng.uniform(-100, 100, (n, 3))
        targets = rng.uniform(-100, 100, (n, 3))
        t0 = time.perf_counter()
        assign = a.lbap_assign(current, targets)
        elapsed = time.perf_counter() - t0
        assert len(set(assign)) == n
        assert elapsed < 30.0, f"LBAP took too long: {elapsed:.2f}s"


# ---------------------------------------------------------------------------
# Demo show generator
# ---------------------------------------------------------------------------

class TestDemoGenerator:
    def test_generates_correct_drone_count(self):
        from src.choreo_parser.demo_generator import generate_demo_show
        for n in [10, 50, 100]:
            show = generate_demo_show(n)
            assert len(show["trajectories"]) == n
            assert len(show["light_cues"]) == n

    def test_metadata_fields(self):
        from src.choreo_parser.demo_generator import generate_demo_show
        show = generate_demo_show(20)
        meta = show["metadata"]
        for key in ("version", "title", "duration", "drone_count"):
            assert key in meta
        assert meta["duration"] > 0

    def test_waypoints_have_required_keys(self):
        from src.choreo_parser.demo_generator import generate_demo_show
        show = generate_demo_show(10)
        for traj in show["trajectories"]:
            for wp in traj["waypoints"]:
                for k in ("t", "x", "y", "z"):
                    assert k in wp

    def test_light_cues_have_required_keys(self):
        from src.choreo_parser.demo_generator import generate_demo_show
        show = generate_demo_show(10)
        for lc in show["light_cues"]:
            for kf in lc["keyframes"]:
                for k in ("t", "r", "g", "b"):
                    assert k in kf
                assert 0 <= kf["r"] <= 255
                assert 0 <= kf["g"] <= 255
                assert 0 <= kf["b"] <= 255

    def test_skyc_bytes_is_valid_zip(self):
        import zipfile, io
        from src.choreo_parser.demo_generator import build_skyc_bytes
        data = build_skyc_bytes(10)
        assert len(data) > 100
        buf = io.BytesIO(data)
        assert zipfile.is_zipfile(buf)


# ---------------------------------------------------------------------------
# Trajectory interpolation
# ---------------------------------------------------------------------------

class TestTrajectoryInterpolation:
    def _make_interpolator(self, waypoints):
        """Import and instantiate TrajectoryInterpolator if available."""
        try:
            from fast_tensor_sim.drone_math import TrajectoryInterpolator
            return TrajectoryInterpolator(waypoints)
        except ImportError:
            pytest.skip("TrajectoryInterpolator not available")

    def test_position_at_waypoint(self):
        wps = [
            {"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0},
            {"t": 10.0, "x": 10.0, "y": 0.0, "z": 5.0},
        ]
        interp = self._make_interpolator(wps)
        x, y, z = interp.position_at(0.0)
        assert abs(x) < 0.5 and abs(z) < 0.5

        x, y, z = interp.position_at(10.0)
        assert abs(x - 10.0) < 0.5 and abs(z - 5.0) < 0.5

    def test_midpoint_interpolation(self):
        wps = [
            {"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0},
            {"t": 10.0, "x": 10.0, "y": 0.0, "z": 0.0},
        ]
        interp = self._make_interpolator(wps)
        x, y, z = interp.position_at(5.0)
        # Should be close to midpoint
        assert 4.0 < x < 6.0

    def test_duration(self):
        wps = [{"t": 0.0, "x": 0, "y": 0, "z": 0}, {"t": 60.0, "x": 10, "y": 0, "z": 10}]
        interp = self._make_interpolator(wps)
        assert abs(interp.duration() - 60.0) < 0.01


# ---------------------------------------------------------------------------
# Geofence
# ---------------------------------------------------------------------------

class TestGeofence:
    def _get_engine(self):
        try:
            from src.fleet_manager.geofence import GeofenceEngine, GeofenceZone
            return GeofenceEngine, GeofenceZone
        except ImportError:
            pytest.skip("GeofenceEngine not available")

    def test_safe_position(self):
        GE, GZ = self._get_engine()
        engine = GE()
        zone = GZ(zone_id=0, center_x=0, center_y=0, center_z=20, radius_m=50.0, min_alt=0, max_alt=60, zone_type="SAFE")
        engine.add_zone(zone)
        status = engine.check_drone(0, 0.0, 0.0, 20.0)
        assert "VIOLATION" not in str(status).upper()

    def test_violation_outside_radius(self):
        # SAFE zone = drone must stay inside. Flying outside is a hard violation.
        GE, GZ = self._get_engine()
        from src.fleet_manager.geofence import GeofenceStatus
        engine = GE()
        zone = GZ(zone_id=0, center_x=0, center_y=0, center_z=20, radius_m=10.0, min_alt=0, max_alt=60, zone_type="SAFE")
        engine.add_zone(zone)
        status = engine.check_drone(0, 100.0, 100.0, 20.0)
        assert status == GeofenceStatus.VIOLATION

    def test_vectorized_check(self):
        GE, GZ = self._get_engine()
        engine = GE()
        zone = GZ(zone_id=0, center_x=0, center_y=0, center_z=20, radius_m=30.0, min_alt=0, max_alt=60, zone_type="HARD")
        engine.add_zone(zone)
        positions = np.array([
            [0.0, 0.0, 20.0],   # safe
            [100.0, 0.0, 20.0], # violation
            [5.0, 5.0, 30.0],   # safe
        ])
        results = engine.check_swarm(positions)
        assert len(results) == 3


# ---------------------------------------------------------------------------
# FlockWave protocol
# ---------------------------------------------------------------------------

class TestFlockwaveProtocol:
    def test_drone_info_serialization(self):
        try:
            from src.protocols.flockwave import DroneInfo, DroneStatus
        except ImportError:
            pytest.skip("flockwave not available")
        d = DroneInfo(
            id=0, x=1.0, y=2.0, z=15.0,
            vx=0.1, vy=-0.2, vz=0.0,
            roll=0.01, pitch=-0.01, yaw=1.5,
            r=255, g=128, b=0,
            battery=0.875, status=DroneStatus.FLYING, rtk_fix=True
        )
        data = d.model_dump()
        assert data["id"] == 0
        assert data["battery"] == pytest.approx(0.875, abs=0.01)

    def test_telemetry_broadcast_serializable(self):
        try:
            from src.protocols.flockwave import DroneInfo, DroneStatus, make_telemetry
        except ImportError:
            pytest.skip("flockwave not available")
        import json
        drones = [
            DroneInfo(id=i, x=float(i), y=0.0, z=10.0,
                      vx=0.0, vy=0.0, vz=0.0,
                      roll=0.0, pitch=0.0, yaw=0.0,
                      r=255, g=0, b=i * 5,
                      battery=0.9, status=DroneStatus.FLYING, rtk_fix=True)
            for i in range(5)
        ]
        msg = make_telemetry(drones)
        # make_telemetry may return a Pydantic model or a plain dict
        raw = json.dumps(msg.model_dump() if hasattr(msg, "model_dump") else msg)
        parsed = json.loads(raw)
        assert parsed["type"] == "TEL"
        assert len(parsed["drones"]) == 5


# ---------------------------------------------------------------------------
# Simulation core smoke test
# ---------------------------------------------------------------------------

class TestSimulatorCore:
    def _get_sim(self, n=10):
        try:
            from fast_tensor_sim.sim_core import DroneSwarmSimulator
            return DroneSwarmSimulator(n)
        except ImportError:
            pytest.skip("DroneSwarmSimulator not available")

    def test_initialization(self):
        sim = self._get_sim(20)
        assert sim.positions.shape == (20, 3)
        assert sim.velocities.shape == (20, 3)
        assert sim.battery.shape == (20,)
        assert (sim.battery > 90).all()

    def test_set_drone_count(self):
        sim = self._get_sim(10)
        sim.set_drone_count(50)
        assert sim.positions.shape == (50, 3)

    def test_telemetry_format(self):
        sim = self._get_sim(5)
        telemetry = sim.get_telemetry()
        assert len(telemetry) == 5
        for d in telemetry:
            for key in ("id", "x", "y", "z", "vx", "vy", "vz", "r", "g", "b", "battery", "status"):
                assert key in d

    def test_load_demo_show(self):
        from src.choreo_parser.demo_generator import generate_demo_show
        sim = self._get_sim(20)
        show = generate_demo_show(20)
        result = sim.load_show(show)
        assert result is True
        assert sim.show_loaded

    def test_physics_step_does_not_crash(self):
        sim = self._get_sim(10)
        for _ in range(100):
            sim._physics_step()
        assert not np.any(np.isnan(sim.positions))
        assert not np.any(np.isnan(sim.velocities))

    def test_arm_and_state_transition(self):
        from fast_tensor_sim.sim_core import DroneStatus
        sim = self._get_sim(5)
        sim.arm_all()
        assert (sim.status == DroneStatus.ARMING).all() or (sim.status != DroneStatus.IDLE).any()
