"""PID controllers for the AUV simulation."""

from __future__ import annotations

from typing import Tuple


def pitch_pid(
    theta_command: float,
    depth: float,
    theta: float,
    pitch_rate: float,
    integrator_state: float,
    dt: float,
) -> Tuple[float, float]:
    """Compute elevator deflection using pitch angle PID control."""

    Kp = 3.0
    Ki = 1.0
    Kq = 2.0

    error_theta = theta_command - theta
    integrator = integrator_state + dt * error_theta
    elevator = -(Kp * error_theta + Ki * integrator - Kq * pitch_rate)
    return elevator, integrator


def yaw_pid(
    psi_command: float,
    psi: float,
    yaw_rate: float,
    integrator_state: float,
    dt: float,
) -> Tuple[float, float]:
    """Compute rudder deflection using yaw PID control."""

    Kp = 4.0
    Ki = 0.1
    Kr = 3.0

    error_psi = psi_command - psi
    integrator = integrator_state + dt * error_psi
    rudder = -(Kp * error_psi + Ki * integrator - Kr * yaw_rate)
    return rudder, integrator

