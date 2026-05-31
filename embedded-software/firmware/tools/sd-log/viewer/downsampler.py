"""Min-max decimation for efficient rendering of large time series."""

from __future__ import annotations

import numpy as np


def downsample_minmax(
    t: np.ndarray,
    y: np.ndarray,
    t_min: float,
    t_max: float,
    n_pixels: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Downsample a time series to ~2*n_pixels points using min-max per bucket.

    Returns (t_out, y_out) containing the min and max values per bucket,
    preserving visual peaks and valleys.
    """
    if n_pixels < 2:
        n_pixels = 2

    # Find visible slice via binary search
    i0 = int(np.searchsorted(t, t_min, side="left"))
    i1 = int(np.searchsorted(t, t_max, side="right"))
    t_vis = t[i0:i1]
    y_vis = y[i0:i1]
    n = len(t_vis)

    if n <= 2 * n_pixels:
        return t_vis, y_vis

    bucket_size = n // n_pixels
    n_trim = bucket_size * n_pixels
    t_trim = t_vis[:n_trim].reshape(n_pixels, bucket_size)
    y_trim = y_vis[:n_trim].reshape(n_pixels, bucket_size)

    # For each bucket, find min and max indices
    min_idx = np.argmin(y_trim, axis=1)
    max_idx = np.argmax(y_trim, axis=1)

    # Build output: 2 points per bucket (min and max, in temporal order)
    t_out = np.empty(2 * n_pixels, dtype=np.float64)
    y_out = np.empty(2 * n_pixels, dtype=np.float64)

    for i in range(n_pixels):
        mi, ma = min_idx[i], max_idx[i]
        if mi <= ma:
            t_out[2 * i] = t_trim[i, mi]
            y_out[2 * i] = y_trim[i, mi]
            t_out[2 * i + 1] = t_trim[i, ma]
            y_out[2 * i + 1] = y_trim[i, ma]
        else:
            t_out[2 * i] = t_trim[i, ma]
            y_out[2 * i] = y_trim[i, ma]
            t_out[2 * i + 1] = t_trim[i, mi]
            y_out[2 * i + 1] = y_trim[i, mi]

    return t_out, y_out
