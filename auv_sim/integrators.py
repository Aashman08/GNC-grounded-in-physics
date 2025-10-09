"""Integration utilities for the simulation."""

from __future__ import annotations

from typing import Callable, Sequence

import numpy as np


def runge_kutta4(
    derivative_fn: Callable[[Sequence[float], Sequence[float]], np.ndarray],
    state: np.ndarray,
    inputs: Sequence[float],
    dt: float,
) -> np.ndarray:
    """Classical fourth-order Runge-Kutta integration step."""

    state = np.asarray(state, dtype=float)

    f1 = derivative_fn(state, inputs)  # slope at beginning of interval
    x1 = state + (dt / 2.0) * f1

    f2 = derivative_fn(x1, inputs)  # slope at midpoint using k1
    x2 = state + (dt / 2.0) * f2

    f3 = derivative_fn(x2, inputs)  # slope at midpoint using k2
    x3 = state + dt * f3

    f4 = derivative_fn(x3, inputs)  # slope at end of interval
    return state + (dt / 6.0) * (f1 + 2 * f2 + 2 * f3 + f4)

