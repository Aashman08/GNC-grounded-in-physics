"""Plotting utilities for simulation results."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np


def plot_longitudinal_response(
    time: Iterable[float],
    theta_command: Iterable[float],
    theta: Iterable[float],
    elevator: Iterable[float],
    save_dir: Path,
) -> Path:
    """Generate and save the longitudinal response plot."""

    time_arr = np.asarray(list(time))
    theta_cmd_arr = np.asarray(list(theta_command))
    theta_arr = np.asarray(list(theta))
    elevator_arr = np.asarray(list(elevator))

    plt.close("all")
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    axes[0].plot(time_arr, np.rad2deg(theta_cmd_arr), ":r", linewidth=3, label="theta_cmd")
    axes[0].plot(time_arr, np.rad2deg(theta_arr), linewidth=3, label="theta")
    axes[0].set_ylabel("theta (deg)")
    axes[0].legend(loc="best")

    axes[1].plot(time_arr, np.rad2deg(elevator_arr), linewidth=3)
    axes[1].set_xlabel("time (sec)")
    axes[1].set_ylabel("delta_e (deg)")

    fig.tight_layout()

    save_path = save_dir / "longitudinal_response.png"
    fig.savefig(save_path)
    plt.close(fig)
    return save_path

