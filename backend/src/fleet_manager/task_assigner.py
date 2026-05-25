"""
Task assignment algorithms for drone swarm formation transitions.

Hungarian Algorithm: minimizes total distance (sum of all assignments)
LBAP: minimizes the maximum individual distance (bottleneck minimization)
      This equalizes battery drain across the fleet.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
from scipy.optimize import linear_sum_assignment

log = logging.getLogger(__name__)


class TaskAssigner:
    """
    Provides two assignment strategies for transitioning a swarm from its
    current positions to a new set of target positions:

    1. **Hungarian** (scipy) – optimal total-distance assignment.
    2. **LBAP** (Bottleneck) – minimises the maximum single assignment
       distance, equalising battery usage across the fleet.
    """

    # ------------------------------------------------------------------
    # Cost matrix
    # ------------------------------------------------------------------

    def compute_cost_matrix(
        self,
        positions_a: np.ndarray,
        positions_b: np.ndarray,
    ) -> np.ndarray:
        """
        Compute pairwise Euclidean distance matrix between two position sets.

        Parameters
        ----------
        positions_a : np.ndarray  shape (N, 3)
            Source positions.
        positions_b : np.ndarray  shape (M, 3)
            Target positions.

        Returns
        -------
        np.ndarray  shape (N, M)
            cost_matrix[i, j] = Euclidean distance from a[i] to b[j].
        """
        a = np.asarray(positions_a, dtype=np.float64)
        b = np.asarray(positions_b, dtype=np.float64)

        # Broadcast: (N, 1, 3) - (1, M, 3) → (N, M, 3)
        diff = a[:, np.newaxis, :] - b[np.newaxis, :, :]
        return np.sqrt(np.sum(diff ** 2, axis=-1))

    # ------------------------------------------------------------------
    # Hungarian assignment
    # ------------------------------------------------------------------

    def hungarian_assign(
        self,
        current_positions: np.ndarray,
        target_positions: np.ndarray,
    ) -> np.ndarray:
        """
        Optimal total-distance assignment via the Hungarian algorithm.

        Uses scipy.optimize.linear_sum_assignment on the Euclidean cost
        matrix.  Minimises ∑ dist(current[i], target[assignment[i]]).

        Parameters
        ----------
        current_positions : np.ndarray  shape (N, 3)
        target_positions  : np.ndarray  shape (N, 3)

        Returns
        -------
        np.ndarray  shape (N,)  dtype int
            assignment[i] = index of the target assigned to drone i.
        """
        cost = self.compute_cost_matrix(current_positions, target_positions)
        row_ind, col_ind = linear_sum_assignment(cost)
        # Reconstruct full assignment vector (row_ind is already sorted 0..N-1)
        assignment = np.empty(len(row_ind), dtype=np.intp)
        assignment[row_ind] = col_ind
        return assignment

    # ------------------------------------------------------------------
    # LBAP (Bottleneck) assignment
    # ------------------------------------------------------------------

    def lbap_assign(
        self,
        current_positions: np.ndarray,
        target_positions: np.ndarray,
    ) -> np.ndarray:
        """
        Linear Bottleneck Assignment Problem (LBAP).

        Finds the perfect matching that minimises the *maximum* individual
        assignment distance – the bottleneck cost – using binary search over
        sorted unique edge weights.

        Algorithm
        ---------
        1. Compute full Euclidean cost matrix C (N×N).
        2. Collect and sort all unique cost values.
        3. Binary-search for the smallest threshold τ such that the binary
           matrix B[i,j] = (C[i,j] <= τ) admits a perfect matching.
        4. Solve the restricted assignment on B to obtain the assignment.

        A perfect matching exists in B when scipy's linear_sum_assignment
        on the inverted binary matrix (0 where allowed, ∞ where forbidden)
        achieves total cost 0 on the identity-equivalent check.

        Parameters
        ----------
        current_positions : np.ndarray  shape (N, 3)
        target_positions  : np.ndarray  shape (N, 3)

        Returns
        -------
        np.ndarray  shape (N,)  dtype int
            assignment[i] = index of the target assigned to drone i.
        """
        cost = self.compute_cost_matrix(current_positions, target_positions)
        n = cost.shape[0]

        if n == 0:
            return np.array([], dtype=np.intp)

        if n == 1:
            return np.array([0], dtype=np.intp)

        # All unique cost values, sorted ascending
        unique_costs = np.unique(cost)

        # Binary search for minimum feasible threshold
        lo, hi = 0, len(unique_costs) - 1
        best_threshold_idx = hi  # worst-case: use maximum cost

        while lo <= hi:
            mid = (lo + hi) // 2
            threshold = unique_costs[mid]

            if self._has_perfect_matching(cost, threshold, n):
                best_threshold_idx = mid
                hi = mid - 1
            else:
                lo = mid + 1

        threshold = unique_costs[best_threshold_idx]

        # Build restricted cost matrix: 0 where allowed, large penalty where not
        BIG = 1e12
        restricted = np.where(cost <= threshold, cost, BIG)

        row_ind, col_ind = linear_sum_assignment(restricted)
        assignment = np.empty(n, dtype=np.intp)
        assignment[row_ind] = col_ind
        return assignment

    @staticmethod
    def _has_perfect_matching(cost: np.ndarray, threshold: float, n: int) -> bool:
        """
        Check whether the binary matrix cost <= threshold admits a perfect
        matching.

        We build a 0/BIG cost matrix and run the Hungarian algorithm.
        A perfect matching exists iff the total cost < BIG (i.e. no forbidden
        edge was used).
        """
        BIG = 1e12
        restricted = np.where(cost <= threshold, 0.0, BIG)
        _, col_ind = linear_sum_assignment(restricted)
        total = sum(restricted[i, col_ind[i]] for i in range(n))
        return total < BIG * 0.5

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def assignment_stats(
        self,
        assignment: np.ndarray,
        cost_matrix: np.ndarray,
    ) -> dict[str, float]:
        """
        Compute descriptive statistics for an assignment.

        Parameters
        ----------
        assignment : np.ndarray  shape (N,)
            assignment[i] = target index for drone i.
        cost_matrix : np.ndarray  shape (N, N)
            Pairwise cost matrix.

        Returns
        -------
        dict with keys: max_cost, min_cost, mean_cost, total_cost
        """
        costs = np.array(
            [cost_matrix[i, assignment[i]] for i in range(len(assignment))],
            dtype=np.float64,
        )
        return {
            "max_cost": float(np.max(costs)),
            "min_cost": float(np.min(costs)),
            "mean_cost": float(np.mean(costs)),
            "total_cost": float(np.sum(costs)),
        }

    # ------------------------------------------------------------------
    # Full transition plan
    # ------------------------------------------------------------------

    def transition_plan(
        self,
        current_pos: np.ndarray,
        target_pos: np.ndarray,
        method: str = "lbap",
    ) -> dict[str, Any]:
        """
        Compute a full transition plan from current to target positions.

        Parameters
        ----------
        current_pos : np.ndarray  shape (N, 3)
        target_pos  : np.ndarray  shape (N, 3)
        method : str
            'lbap' (default) or 'hungarian'

        Returns
        -------
        dict with keys:
            method        – algorithm used
            assignment    – list of target indices
            stats         – assignment statistics dict
            drone_moves   – list of {drone_id, from, to, distance} dicts
        """
        current_pos = np.asarray(current_pos, dtype=np.float64)
        target_pos = np.asarray(target_pos, dtype=np.float64)

        if method == "hungarian":
            assignment = self.hungarian_assign(current_pos, target_pos)
        else:
            assignment = self.lbap_assign(current_pos, target_pos)

        cost_matrix = self.compute_cost_matrix(current_pos, target_pos)
        stats = self.assignment_stats(assignment, cost_matrix)

        drone_moves = []
        for i, j in enumerate(assignment):
            drone_moves.append(
                {
                    "drone_id": int(i),
                    "from": current_pos[i].tolist(),
                    "to": target_pos[int(j)].tolist(),
                    "distance": float(cost_matrix[i, int(j)]),
                }
            )

        log.info(
            "Transition plan (%s): N=%d, max_dist=%.2fm, total_dist=%.2fm",
            method,
            len(assignment),
            stats["max_cost"],
            stats["total_cost"],
        )

        return {
            "method": method,
            "assignment": assignment.tolist(),
            "stats": stats,
            "drone_moves": drone_moves,
        }
