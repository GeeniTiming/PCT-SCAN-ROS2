"""CPU-only PCT tomogram implementation for RK3588 and desktop CPUs."""

import time

import numpy as np

from .cpu_kernels import inflate, scatter_points, traversability


class Tomogram(object):
    def __init__(self, cfg):
        self.resolution = cfg.map.resolution
        self.slice_dh = cfg.map.slice_dh
        self.half_trav_k_size = int(cfg.trav.kernel_size / 2)
        self.interval_min = cfg.trav.interval_min
        self.interval_free = cfg.trav.interval_free
        self.step_stand = 1.2 * self.resolution * np.tan(cfg.trav.slope_max)
        self.step_cross = cfg.trav.step_max
        self.standable_th = int(
            cfg.trav.standable_ratio * (2 * self.half_trav_k_size + 1) ** 2
        ) - 1
        self.cost_barrier = float(cfg.trav.cost_barrier)
        self.safe_margin = cfg.trav.safe_margin
        self.inflation = cfg.trav.inflation
        self.half_inf_k_size = int(
            (self.safe_margin + self.inflation) / self.resolution
        )

    def initMappingEnv(self, center, map_dim_x, map_dim_y, n_slice_init,
                       slice_h0):
        self.center = np.asarray(center, dtype=np.float32)
        self.map_dim_x = int(map_dim_x)
        self.map_dim_y = int(map_dim_y)
        self.n_slice_init = int(n_slice_init)
        self.slice_h0 = float(slice_h0)
        shape = (self.n_slice_init, self.map_dim_x, self.map_dim_y)
        self.layers_g = np.empty(shape, dtype=np.float32)
        self.layers_c = np.empty(shape, dtype=np.float32)
        self.grad_mag_sq = np.empty(shape, dtype=np.float32)
        self.grad_mag_max = np.empty(shape, dtype=np.float32)
        axis = (np.arange(-self.half_inf_k_size, self.half_inf_k_size + 1)
                * self.resolution)
        distance = np.hypot(axis[:, None], axis[None, :])
        self.inf_table = np.clip(
            1.0 - (distance - self.inflation)
            / (self.safe_margin + self.resolution), 0.0, 1.0
        ).astype(np.float32)

    def point2map(self, points):
        points = np.asarray(points, dtype=np.float32)
        points = points[np.isfinite(points).all(axis=1)]
        self.layers_g.fill(-1e6)
        self.layers_c.fill(1e6)
        self.grad_mag_sq.fill(0.0)
        self.grad_mag_max.fill(0.0)

        started = time.perf_counter()
        scatter_points(points, self.center, self.layers_g, self.layers_c,
                       self.resolution, self.slice_h0, self.slice_dh)
        diff_x_sq = np.maximum(
            (self.layers_g[:, 1:-1, :] - self.layers_g[:, :-2, :]) ** 2,
            (self.layers_g[:, 1:-1, :] - self.layers_g[:, 2:, :]) ** 2)
        diff_y_sq = np.maximum(
            (self.layers_g[:, :, 1:-1] - self.layers_g[:, :, :-2]) ** 2,
            (self.layers_g[:, :, 1:-1] - self.layers_g[:, :, 2:]) ** 2)
        self.grad_mag_sq[:, 1:-1, 1:-1] = (
            diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :])
        self.grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
            diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :])
        interval = self.layers_c - self.layers_g
        map_ms = (time.perf_counter() - started) * 1000.0

        started = time.perf_counter()
        trav_cost = traversability(
            interval, self.grad_mag_sq, self.grad_mag_max,
            self.half_trav_k_size, self.interval_min, self.interval_free,
            self.step_cross, self.step_stand, self.standable_th,
            self.cost_barrier)
        inflated_cost = inflate(trav_cost, self.inf_table, self.half_inf_k_size)
        trav_ms = (time.perf_counter() - started) * 1000.0

        started = time.perf_counter()
        simplified = [0]
        if self.n_slice_init > 1:
            lower, middle = 0, 1
            height_difference = self.layers_g[1:] - self.layers_g[:-1]
            while middle < self.n_slice_init - 2:
                unique = (
                    ((self.layers_g[middle] - self.layers_g[lower] > 0)
                     | (inflated_cost[lower] > inflated_cost[middle]))
                    & (height_difference[middle] > 0)
                    & (inflated_cost[middle] < self.cost_barrier))
                if np.any(unique):
                    simplified.append(middle)
                    lower = middle
                middle += 1
            simplified.append(middle)

        selected_cost = inflated_cost[simplified]
        selected_ground = self.layers_g[simplified]
        selected_ceiling = self.layers_c[simplified]
        gradient_x = np.zeros_like(selected_ground)
        gradient_y = np.zeros_like(selected_ground)
        gradient_x[:, 1:-1, :] = (
            selected_cost[:, 2:, :] - selected_cost[:, :-2, :])
        gradient_y[:, :, 1:-1] = (
            selected_cost[:, :, 2:] - selected_cost[:, :, :-2])
        selected_ground = np.where(
            selected_ground > -1e6, selected_ground, np.nan)
        selected_ceiling = np.where(
            selected_ceiling < 1e6, selected_ceiling, np.nan)
        simplify_ms = (time.perf_counter() - started) * 1000.0
        timing = {"t_map": map_ms, "t_trav": trav_ms,
                  "t_simp": simplify_ms}
        return (selected_cost, gradient_x, gradient_y, selected_ground,
                selected_ceiling, timing)
