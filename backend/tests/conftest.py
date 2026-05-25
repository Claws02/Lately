"""Pytest configuration — set up sys.path so all source modules are importable."""
import sys
import os

# Backend source
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
# Simulation source
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "simulation"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "simulation", "fast_tensor_sim"))
