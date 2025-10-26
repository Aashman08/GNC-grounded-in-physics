"""Control allocation for X-tail configuration."""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .hydroplane import Hydroplane


class XTailAllocator:
    """Allocates desired body torques to four X-tail fin deflections.

    Uses numerical effectiveness matrix and weighted least-squares solve.
    """

    def __init__(
        self,
        fins: Tuple[Hydroplane, Hydroplane, Hydroplane, Hydroplane],
        delta_max: float = np.deg2rad(30.0),
        lambda_reg: float = 1e1,  # Much larger regularization for stability
    ):
        """Initialize the allocator.

        Parameters
        ----------
        fins : tuple of Hydroplane
            The four hydroplanes (upper-right, lower-right, lower-left, upper-left)
        delta_max : float
            Maximum fin deflection magnitude [rad]
        lambda_reg : float
            Tikhonov regularization parameter for WLS
        """
        self.fins = fins
        self.delta_max = delta_max
        self.lambda_reg = lambda_reg

    def compute_effectiveness(
        self, rho: float, velocity: np.ndarray, angular_vel: np.ndarray
    ) -> np.ndarray:
        """Compute the local effectiveness matrix B = ∂[K,M,N]/∂δ.

        Uses numerical differentiation with small perturbations.

        Parameters
        ----------
        rho : float
            Water density [kg/m^3]
        velocity : np.ndarray
            Body-frame linear velocity [u, v, w] [m/s]
        angular_vel : np.ndarray
            Body-frame angular velocity [p, q, r] [rad/s]

        Returns
        -------
        B : np.ndarray
            Effectiveness matrix, shape (3, 4)
            Rows: [K, M, N] (roll, pitch, yaw moments)
            Cols: [fin0, fin1, fin2, fin3] deflections
        """
        # Store original deflections
        original_deltas = [f.delta for f in self.fins]

        # Baseline: compute torques with all fins at zero
        for f in self.fins:
            f.delta = 0.0

        tau_baseline = np.zeros(3)
        for f in self.fins:
            _, moment = f.compute_forces(rho, velocity, angular_vel)
            tau_baseline += moment

        # Numerical Jacobian via perturbation
        B = np.zeros((3, 4))
        eps = np.deg2rad(1.0)  # small perturbation

        for j, fin in enumerate(self.fins):
            fin.delta = eps

            tau_perturbed = np.zeros(3)
            for f in self.fins:
                _, moment = f.compute_forces(rho, velocity, angular_vel)
                tau_perturbed += moment

            B[:, j] = (tau_perturbed - tau_baseline) / eps
            fin.delta = 0.0

        # Restore original deflections
        for f, orig_delta in zip(self.fins, original_deltas):
            f.delta = orig_delta

        return B

    def allocate(
        self,
        tau_cmd: np.ndarray,
        rho: float,
        velocity: np.ndarray,
        angular_vel: np.ndarray,
    ) -> np.ndarray:
        """Allocate desired torques to fin deflections.

        Solves: min ||B·δ - τ_cmd||² + λ||δ||²  subject to |δ_i| ≤ δ_max

        Parameters
        ----------
        tau_cmd : np.ndarray
            Desired body torques [K, M, N] [N·m]
        rho : float
            Water density [kg/m^3]
        velocity : np.ndarray
            Body-frame linear velocity [u, v, w] [m/s]
        angular_vel : np.ndarray
            Body-frame angular velocity [p, q, r] [rad/s]

        Returns
        -------
        deltas : np.ndarray
            Fin deflections [δ0, δ1, δ2, δ3] [rad], shape (4,)
        """
        B = self.compute_effectiveness(rho, velocity, angular_vel)

        # Weighted least-squares with Tikhonov regularization
        # δ = B^T (B B^T + λ I)^{-1} τ_cmd
        Bt = B.T
        
        # Check for numerical issues
        if np.any(np.isnan(B)) or np.any(np.isinf(B)):
            # Return zero deflections if matrix is invalid
            return np.zeros(4)
        
        try:
            BBt = B @ Bt
            BBt_reg = BBt + self.lambda_reg * np.eye(3)
            deltas = Bt @ np.linalg.solve(BBt_reg, tau_cmd)
        except np.linalg.LinAlgError:
            # If solve fails, return zero deflections
            return np.zeros(4)

        # Saturate to physical limits
        deltas = np.clip(deltas, -self.delta_max, self.delta_max)
        
        # Final check for NaN
        if np.any(np.isnan(deltas)):
            return np.zeros(4)

        return deltas

