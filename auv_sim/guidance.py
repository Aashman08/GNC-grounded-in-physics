"""Guidance setpoint generators for the AUV simulation."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def longitudinal_profile(time: float) -> Tuple[float, float]:
    """Return depth and pitch commands for a simple step schedule."""

    z_command = 0.0
    theta_command = 0.0

    if time >= 1.0:
        theta_command = np.deg2rad(3.0)
    if time >= 11.0:
        theta_command = np.deg2rad(-3.0)
    if time >= 23.0:
        theta_command = np.deg2rad(5.0)
    if time >= 41.0:
        theta_command = np.deg2rad(-5.0)
    if time >= 51.0:
        theta_command = np.deg2rad(2.0)
    if time >= 62.0:
        theta_command = np.deg2rad(-2.0)

    return z_command, theta_command


def lateral_profile(time: float) -> float:
    """Return yaw command for the lateral step schedule."""

    psi_command = 0.0
    if time >= 30.0:
        psi_command = np.deg2rad(5.0)
    return psi_command

