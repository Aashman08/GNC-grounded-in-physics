"""3D hydroplane force and moment calculations for X-tail configuration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class Hydroplane:
    """Represents a single hydroplane control surface."""

    S: float  # planform area [m^2]
    AR: float  # aspect ratio
    pos: np.ndarray  # 3D position in body frame [m]
    N: np.ndarray  # unit normal vector of plane (direction of lift axis)
    delta: float = 0.0  # current deflection angle [rad]

    def compute_forces(
        self, rho: float, velocity: np.ndarray, angular_vel: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute hydrodynamic forces and moments from this hydroplane.

        Based on Lind & Meijer thesis Eqs. 3.4-3.11, 4.17-4.19.

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
        force : np.ndarray
            Force in body frame [Fx, Fy, Fz] [N]
        moment : np.ndarray
            Moment in body frame [Mx, My, Mz] [N·m]
        """
        # Local water velocity at hydroplane (including rotational effects)
        v_r = -(velocity + np.cross(angular_vel, self.pos))
        Vr = np.linalg.norm(v_r) + 1e-9  # avoid division by zero
        vr_unit = v_r / Vr

        # Project velocity onto plane orthogonal to N
        P = np.eye(3) - np.outer(self.N, self.N)
        v_e = P @ v_r

        # Hydrodynamic rudder angle (angle between projected velocity and -x axis)
        base_vec = np.array([-1.0, 0.0, 0.0])
        ve_unit = v_e / (np.linalg.norm(v_e) + 1e-9)
        cos_dh = np.clip(ve_unit @ base_vec, -1.0, 1.0)
        delta_h = np.arccos(cos_dh)

        # Effective angle of attack
        delta_e = self.delta - delta_h

        # Lift and drag coefficients (Eqs. 3.10-3.11)
        CL = 6.13 * self.AR / (2.25 + self.AR)
        CD = CL**2 / (np.pi * self.AR)

        # Lift and drag magnitudes (Eqs. 3.4-3.5)
        L = 0.5 * rho * Vr**2 * self.S * CL * np.cos(delta_e) * np.sin(delta_e)
        D = 0.5 * rho * Vr**2 * self.S * CD * np.sin(delta_e) ** 2

        # Force directions
        F_drag = -D * vr_unit
        F_lift = L * np.cross(self.N, vr_unit)
        F_total = F_drag + F_lift

        # Moment about body origin
        M_total = np.cross(self.pos, F_total)

        return F_total, M_total


def create_x_tail_fins(
    S: float = 0.12,
    AR: float = 3.0,
    x_position: float = -1.8,
    span: float = 0.3,
) -> Tuple[Hydroplane, Hydroplane, Hydroplane, Hydroplane]:
    """Create four hydroplanes in X-tail configuration at 45° dihedral.

    Parameters
    ----------
    S : float
        Planform area of each fin [m^2]
    AR : float
        Aspect ratio of each fin
    x_position : float
        Axial position aft of center of buoyancy [m] (negative = aft)
    span : float
        Half-span distance from centerline [m]

    Returns
    -------
    tuple of Hydroplane
        (upper_right, lower_right, lower_left, upper_left)
    """
    ns = 1.0 / np.sqrt(2.0)  # sine/cosine of 45°

    # Positions (aft of CB, arranged in X pattern)
    pos_ur = np.array([x_position, +span, +span])  # upper-right
    pos_lr = np.array([x_position, +span, -span])  # lower-right
    pos_ll = np.array([x_position, -span, -span])  # lower-left
    pos_ul = np.array([x_position, -span, +span])  # upper-left

    # Normal vectors (45° dihedral)
    N_ur = np.array([0.0, +ns, +ns])
    N_lr = np.array([0.0, +ns, -ns])
    N_ll = np.array([0.0, -ns, -ns])
    N_ul = np.array([0.0, -ns, +ns])

    return (
        Hydroplane(S, AR, pos_ur, N_ur),
        Hydroplane(S, AR, pos_lr, N_lr),
        Hydroplane(S, AR, pos_ll, N_ll),
        Hydroplane(S, AR, pos_ul, N_ul),
    )

