"""PID controllers for X-tail AUV - outputs torque commands."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def pitch_torque_pid(
    theta_command: float,
    depth: float,
    theta: float,
    pitch_rate: float,
    integrator_state: float,
    dt: float,
) -> Tuple[float, float]:
    """Compute desired pitch moment using PID control.

    Returns
    -------
    M_cmd : float
        Desired pitch moment [N·m]
    integrator : float
        Updated integrator state
    """
    Kp = 50.0  # Much smaller for moment control
    Ki = 5.0
    Kq = 20.0

    error_theta = theta_command - theta
    integrator = integrator_state + dt * error_theta
    M_cmd = -(Kp * error_theta + Ki * integrator - Kq * pitch_rate)
    return M_cmd, integrator


def yaw_torque_pid(
    psi_command: float,
    psi: float,
    yaw_rate: float,
    integrator_state: float,
    dt: float,
) -> Tuple[float, float]:
    """Compute desired yaw moment using PID control.

    Returns
    -------
    N_cmd : float
        Desired yaw moment [N·m]
    integrator : float
        Updated integrator state
    """
    Kp = 60.0  # Much smaller
    Ki = 2.0
    Kr = 30.0

    error_psi = psi_command - psi
    integrator = integrator_state + dt * error_psi
    N_cmd = -(Kp * error_psi + Ki * integrator - Kr * yaw_rate)
    return N_cmd, integrator


def roll_torque_pid(
    phi_command: float,
    phi: float,
    roll_rate: float,
    integrator_state: float,
    dt: float,
) -> Tuple[float, float]:
    """Compute desired roll moment using PID control.

    Returns
    -------
    K_cmd : float
        Desired roll moment [N·m]
    integrator : float
        Updated integrator state
    """
    Kp = 30.0  # Much smaller
    Ki = 3.0
    Kp_rate = 15.0

    error_phi = phi_command - phi
    integrator = integrator_state + dt * error_phi
    K_cmd = -(Kp * error_phi + Ki * integrator - Kp_rate * roll_rate)
    return K_cmd, integrator

