"""Validation policy for using localization odometry as a PCT start point."""

from __future__ import annotations

import math


class OdomStartPolicy:
    """Cache odometry and return a safe, fresh XYZ planning start."""

    def __init__(self, timeout_sec, expected_frame, z_offset=0.0):
        self.timeout_sec = float(timeout_sec)
        self.expected_frame = str(expected_frame)
        self.z_offset = float(z_offset)
        self._message = None
        self._received_at_ns = None

    def update(self, message, received_at_ns):
        self._message = message
        self._received_at_ns = int(received_at_ns)

    def resolve(self, now_ns):
        if self._message is None or self._received_at_ns is None:
            return None, "odom_unavailable"

        age_sec = max(0.0, (int(now_ns) - self._received_at_ns) / 1.0e9)
        if self.timeout_sec > 0.0 and age_sec > self.timeout_sec:
            return None, "odom_stale_{:.3f}s".format(age_sec)

        actual_frame = str(self._message.header.frame_id).lstrip("/")
        expected_frame = self.expected_frame.lstrip("/")
        if not actual_frame:
            return None, "odom_frame_empty"
        if actual_frame != expected_frame:
            return None, "odom_frame_{}_expected_{}".format(
                actual_frame, expected_frame
            )

        position = self._message.pose.pose.position
        xyz = (
            float(position.x),
            float(position.y),
            float(position.z) + self.z_offset,
        )
        if not all(math.isfinite(value) for value in xyz):
            return None, "odom_position_non_finite"
        return xyz, "ok"
