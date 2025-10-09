"""Dynamic model for the six-DOF AUV."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class AUVHydroParameters:
    """Container for vehicle hydrodynamic and inertia parameters."""

    rho: float = +1.03e3  # water density [kg/m^3]
    Af: float = +2.85e-2  # frontal area [m^2]
    B: float = +3.08e2  # buoyancy force [N]
    W: float = +2.99e2  # vehicle weight [N]
    Ix: float = +1.77e-1  # roll inertia [kg·m^2]
    Iy: float = +3.45e0  # pitch inertia [kg·m^2]
    Iz: float = +3.45e0  # yaw inertia [kg·m^2]
    xB: float = +0.0e0
    yB: float = +0.0e0
    zB: float = +0.0e0
    xG: float = -0.0e0
    yG: float = +0.0e0
    zG: float = +1.96e-2
    X_du: float = -9.30e-1
    X_wq: float = -3.55e1
    X_qq: float = -1.93e0
    X_vr: float = +3.55e1
    X_rr: float = -1.93e0
    Y_dv: float = -3.55e1
    Y_dr: float = +1.93e0
    Yvav: float = -1.31e3
    Yrar: float = +6.32e-1
    Y_wp: float = +3.55e1
    Y_pq: float = +1.93e0
    Y_uv: float = -2.86e1
    Y_ur: float = +5.22e0
    Z_dw: float = -3.55e1
    Z_dq: float = -1.93e0
    Zwaw: float = -1.31e2
    Zqaq: float = -6.32e-1
    Z_vp: float = -3.55e1
    Z_rp: float = +1.93e0
    Z_uw: float = -2.86e1
    Z_uq: float = -5.22e0
    K_dp: float = -7.04e-2
    Kpap: float = -1.30e-1 * 10
    M_dw: float = -1.93e0
    M_dq: float = -4.88e0
    Mwaw: float = +3.18e0
    Mqaq: float = -1.88e2
    M_vp: float = -1.93e0
    M_rp: float = +4.86e0
    M_uw: float = +2.40e1
    M_uq: float = -2.00e0
    N_dv: float = +1.93e0
    N_dr: float = -4.88e0
    Nvav: float = -3.18e0
    Nrar: float = -9.40e1
    N_wp: float = -1.93e0
    N_pq: float = -4.86e0
    N_uv: float = -2.40e1
    N_ur: float = -2.00e0
    Y_uudr: float = +9.64e0
    Z_uude: float = -9.64e0
    M_uude: float = -6.15e0
    N_uudr: float = -6.15e0
    X_T: float = +9.25e0
    K_T: float = -5.43e-1


def auv_dynamics(
    states: Sequence[float],
    inputs: Sequence[float],
    params: AUVHydroParameters | None = None,
) -> np.ndarray:
    """Compute state derivatives for the AUV.

    Parameters
    ----------
    states
        State vector ``[u, v, w, p, q, r, x, y, z, phi, theta, psi]``.
    inputs
        Control inputs ``[elevator_cmd, rudder_cmd]``.
    params
        Vehicle parameters; defaults to :class:`AUVHydroParameters` values.

    Returns
    -------
    numpy.ndarray
        Derivative vector ``[u_dot, ..., psi_dot]``.
    """

    if params is None:
        params = AUVHydroParameters()

    u, v, w = states[0], states[1], states[2]  # linear body-frame velocities
    p, q, r = states[3], states[4], states[5]  # angular rates
    _x, _y, _z = states[6], states[7], states[8]  # placeholder to keep full 12-state convention
    phi, theta, psi = states[9], states[10], states[11]  # Euler attitude angles

    # Extract inputs for clarity
    dele_ac, delr_ac = inputs[0], inputs[1]  # elevator and rudder commands

    # Shorthand references to keep expressions readable
    m = params.W / 9.8
    rho = params.rho
    Af = params.Af

    Cd = 0.193 * (max(abs(u), 1e-6)) ** (-0.14)  # empirical drag coefficient with floor for stability
    Xuau = -0.5 * rho * Af * Cd * 1.5  # surge damping contribution

    M = np.array(
        [
            [m - params.X_du, 0.0, 0.0, 0.0, m * params.zG, -m * params.yG],
            [
                0.0,
                m - params.Y_dv,
                0.0,
                -m * params.zG,
                0.0,
                m * params.xG - params.Y_dr,
            ],
            [
                0.0,
                0.0,
                m - params.Z_dw,
                m * params.yG,
                -m * params.xG - params.Z_dq,
                0.0,
            ],
            [0.0, -m * params.zG, m * params.yG, params.Ix - params.K_dp, 0.0, 0.0],
            [m * params.zG, 0.0, -m * params.xG - params.M_dw, 0.0, params.Iy - params.M_dq, 0.0],
            [
                -m * params.yG,
                m * params.xG - params.N_dv,
                0.0,
                0.0,
                0.0,
                params.Iz - params.N_dr,
            ],
        ]
    )  # coupled inertia + added-mass matrix

    F1 = (
        -(params.W - params.B) * math.sin(theta)
        + Xuau * abs(u) * u
        + (params.X_wq - m) * w * q
        + (params.X_qq + m * params.xG) * q**2
        + (params.X_vr + m) * v * r
        + (params.X_rr + m * params.xG) * r**2
        - m * params.yG * p * q
        - m * params.zG * p * r
        + params.X_T
    )

    F2 = (
        (params.W - params.B) * math.cos(theta) * math.sin(phi)
        + params.Yvav * abs(v) * v
        + params.Yrar * abs(r) * r
        + m * params.yG * r**2
        + (params.Y_ur - m) * u * r
        + (params.Y_wp + m) * w * p
        + (params.Y_pq - m * params.xG) * p * q
        + params.Y_uv * u * v
        + m * params.yG * p**2
        - m * params.zG * q * r
        + params.Y_uudr * u**2 * delr_ac
    )

    F3 = (
        (params.W - params.B) * math.cos(theta) * math.cos(phi)
        + params.Zwaw * abs(w) * w
        + params.Zqaq * abs(q) * q
        + (params.Z_uq + m) * u * q
        + (params.Z_vp - m) * v * p
        + (params.Z_rp - m * params.xG) * r * p
        + params.Z_uw * u * w
        + m * params.zG * (q**2 + p**2)
        - m * params.yG * r * q
        + params.Z_uude * u**2 * dele_ac
    )

    F4 = (
        (params.yG * params.W - params.yB * params.B) * math.cos(theta) * math.cos(phi)
        - (params.zG * params.W - params.zB * params.B) * math.cos(theta) * math.sin(phi)
        + params.Kpap * abs(p) * p
        - (params.Iz - params.Iy) * q * r
        + m * params.yG * (u * q - v * p)
        - m * params.zG * (w * p - u * r)
        + params.K_T
    )

    F5 = (
        -(params.zG * params.W - params.zB * params.B) * math.sin(theta)
        - (params.xG * params.W - params.xB * params.B) * math.cos(theta) * math.cos(phi)
        + params.Mwaw * abs(w) * w
        + params.Mqaq * abs(q) * q
        + (params.M_uq - m * params.xG) * u * q
        + (params.M_vp + m * params.xG) * v * p
        + (params.M_rp - (params.Ix - params.Iz)) * r * p
        + m * params.zG * (v * r - q * w)
        + params.M_uw * u * w
        + params.M_uude * u**2 * dele_ac
    )

    F6 = (
        (params.xG * params.W - params.xB * params.B) * math.cos(theta) * math.sin(phi)
        + (params.yG * params.W - params.yB * params.B) * math.sin(theta)
        + params.Nvav * abs(v) * v
        + params.Nrar * abs(r) * r
        + (params.N_ur - m * params.xG) * u * r
        + (params.N_wp + m * params.xG) * w * p
        + (params.N_pq - (params.Iy - params.Ix)) * p * q
        - m * params.yG * (v * r - w * q)
        + params.N_uv * u * v
        + params.N_uudr * u**2 * delr_ac
    )

    forces = np.array([F1, F2, F3, F4, F5, F6])  # generalized force/moment vector

    accelerations = np.linalg.solve(M, forces)  # solve for body accelerations

    x_dot = (
        math.cos(psi) * math.cos(theta) * u
        + (-math.sin(psi) * math.cos(phi) + math.cos(psi) * math.sin(theta) * math.sin(phi)) * v
        + (math.sin(psi) * math.sin(phi) + math.cos(psi) * math.cos(phi) * math.sin(theta)) * w
    )  # inertial x velocity

    y_dot = (
        math.sin(psi) * math.cos(theta) * u
        + (math.cos(psi) * math.cos(phi) + math.sin(phi) * math.sin(theta) * math.sin(psi)) * v
        + (-math.cos(psi) * math.sin(phi) + math.sin(theta) * math.sin(psi) * math.cos(phi)) * w
    )  # inertial y velocity

    z_dot = (
        -math.sin(theta) * u
        + math.cos(theta) * math.sin(phi) * v
        + math.cos(theta) * math.cos(phi) * w
    )  # inertial z velocity

    phi_dot = p + math.sin(phi) * math.tan(theta) * q + math.cos(phi) * math.tan(theta) * r  # roll rate
    theta_dot = math.cos(phi) * q - math.sin(phi) * r  # pitch rate
    psi_dot = (math.sin(phi) / math.cos(theta)) * q + (math.cos(phi) / math.cos(theta)) * r  # yaw rate

    kinematics = np.array([x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot])  # position/attitude derivatives

    return np.concatenate((accelerations, kinematics))  # 12-element state derivative

