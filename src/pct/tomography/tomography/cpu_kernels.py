"""Vectorized NumPy kernels for PCT tomography on ARM64 CPUs."""

import numpy as np


def _round_away_from_zero(values):
    return np.where(values >= 0.0, np.floor(values + 0.5), np.ceil(values - 0.5))


def scatter_points(points, center, layers_g, layers_c, resolution, slice_h0,
                   slice_dh):
    """Populate per-layer ground maxima and ceiling minima in place."""
    n_slices, rows, columns = layers_g.shape
    cell_xy = _round_away_from_zero((points[:, :2] - center) / resolution)
    cell_xy = cell_xy.astype(np.int32)
    cell_xy[:, 0] += rows // 2
    cell_xy[:, 1] += columns // 2
    inside = ((cell_xy[:, 0] >= 0) & (cell_xy[:, 0] < rows)
              & (cell_xy[:, 1] >= 0) & (cell_xy[:, 1] < columns))
    flat_cells = cell_xy[inside, 0] * columns + cell_xy[inside, 1]
    heights = points[inside, 2]
    ground = layers_g.reshape(n_slices, -1)
    ceiling = layers_c.reshape(n_slices, -1)
    for layer in range(n_slices):
        below = heights <= slice_h0 + layer * slice_dh
        np.maximum.at(ground[layer], flat_cells[below], heights[below])
        np.minimum.at(ceiling[layer], flat_cells[~below], heights[~below])


def _window_reduce(source, radius, operation, weights=None):
    _, rows, columns = source.shape
    result = np.zeros_like(source)
    for dx in range(-radius, radius + 1):
        x_start, x_end = max(0, -dx), min(rows, rows - dx)
        for dy in range(-radius, radius + 1):
            y_start, y_end = max(0, -dy), min(columns, columns - dy)
            neighbour = source[:, x_start + dx:x_end + dx,
                               y_start + dy:y_end + dy]
            if weights is not None:
                weight = weights[dx + radius, dy + radius]
                if weight == 0.0:
                    continue
                neighbour = neighbour * weight
            target = result[:, x_start:x_end, y_start:y_end]
            if operation == "sum":
                target += neighbour
            else:
                np.maximum(target, neighbour, out=target)
    return result


def traversability(interval, gradient_sq, gradient_max, radius, interval_min,
                   interval_free, step_cross, step_stand, standable_threshold,
                   cost_barrier):
    base = np.maximum(0.0, 20.0 * (interval_free - interval)).astype(np.float32)
    standable = gradient_sq <= step_stand ** 2
    count = _window_reduce(standable.astype(np.float32), radius, "sum")
    stand_cost = base + 15.0 * gradient_sq / max(step_stand ** 2, 1e-12)
    cross_cost = base + 20.0 * gradient_max / max(step_cross ** 2, 1e-12)
    result = np.where(
        standable, stand_cost,
        np.where((gradient_max <= step_cross ** 2)
                 & (count >= standable_threshold), cross_cost, cost_barrier))
    result[interval < interval_min] = cost_barrier
    return result.astype(np.float32, copy=False)


def inflate(cost, weights, radius):
    return _window_reduce(cost, radius, "max", weights)

