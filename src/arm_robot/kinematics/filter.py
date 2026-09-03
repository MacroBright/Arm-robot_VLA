"""Temporal filters for joint-angle smoothing — eliminates jitter.

EMA:     simple exponential moving average (one parameter, fast).
OneEuro: adaptive low-pass — heavy smoothing when static, low lag when moving.

Reference: Casiez et al. "1€ Filter" (CHI 2012)

Workstream: W1 手势映射 — .claude/workstreams/01-gesture-mapping.md
"""

import time
import numpy as np
from typing import Optional


class AngleFilter:
    """Base class for joint-angle temporal filters."""

    def __call__(self, raw: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError


class EMAFilter(AngleFilter):
    """Exponential Moving Average.

    filtered = alpha * raw + (1 - alpha) * filtered_prev

    Args:
        alpha: [0–1], lower = smoother + more lag.  0.2–0.4 typical for hands.
    """

    def __init__(self, n_joints: int = 16, alpha: float = 0.3):
        self.alpha = alpha
        self._prev: Optional[np.ndarray] = None

    def __call__(self, raw: np.ndarray) -> np.ndarray:
        if self._prev is None:
            self._prev = raw.copy().astype(np.float64)
            return self._prev
        self._prev = self.alpha * raw + (1.0 - self.alpha) * self._prev
        return self._prev.copy()

    def reset(self):
        self._prev = None


class OneEuroFilter(AngleFilter):
    """1€ Filter: adaptive smoothing tuned for human motion.

    - Heavy smoothing on slow/static poses → kills jitter
    - Light smoothing on fast motion → preserves responsiveness

    Args:
        min_cutoff: Hz, minimum cutoff frequency. Lower = more static smoothing.
                    Start at 1.0, adjust 0.5–3.0.
        beta:       Speed coefficient. Higher = quicker response to fast moves.
                    Start at 0.007, adjust 0.001–0.05.
        d_cutoff:   Derivative filter cutoff Hz (usually = min_cutoff).
    """

    def __init__(
        self,
        n_joints: int = 16,
        min_cutoff: float = 1.0,
        beta: float = 0.009,
        d_cutoff: float = 1.0,
    ):
        self.n_joints = n_joints
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff

        self._prev_raw: Optional[np.ndarray] = None
        self._prev_filt: Optional[np.ndarray] = None
        self._prev_deriv: Optional[np.ndarray] = None
        self._prev_time: Optional[float] = None

    def __call__(self, raw: np.ndarray) -> np.ndarray:
        now = time.monotonic()

        if self._prev_raw is None:
            self._prev_raw = raw.copy().astype(np.float64)
            self._prev_filt = raw.copy().astype(np.float64)
            self._prev_deriv = np.zeros(self.n_joints, dtype=np.float64)
            self._prev_time = now
            return self._prev_filt

        dt = now - self._prev_time
        if dt < 1e-6:
            dt = 1.0 / 30.0

        # Derivative (velocity) with its own low-pass
        deriv_raw = (raw - self._prev_raw) / dt
        d_alpha = self._alpha(self.d_cutoff, dt)
        deriv_smooth = d_alpha * deriv_raw + (1.0 - d_alpha) * self._prev_deriv

        # Adaptive cutoff: increases with speed
        speed = np.abs(deriv_smooth)
        cutoff = self.min_cutoff + self.beta * speed

        # Per-joint adaptive low-pass
        result = np.zeros(self.n_joints, dtype=np.float64)
        for i in range(self.n_joints):
            a = self._alpha(cutoff[i], dt)
            result[i] = a * raw[i] + (1.0 - a) * self._prev_filt[i]

        self._prev_raw = raw.copy()
        self._prev_filt = result.copy()
        self._prev_deriv = deriv_smooth.copy()
        self._prev_time = now

        return result

    def reset(self):
        self._prev_raw = None
        self._prev_filt = None
        self._prev_deriv = None
        self._prev_time = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * np.pi * max(cutoff, 1e-6))
        return 1.0 / (1.0 + tau / dt)
