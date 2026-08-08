"""Drive a visual-only Gazebo entity from SCAN's body odometry."""

from __future__ import annotations

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def pose_for_gazebo(source_pose, z_offset):
    """Copy a ROS pose, apply its visual offset and normalize orientation."""
    pose = copy.deepcopy(source_pose)
    pose.position.z += z_offset
    quaternion_norm = math.sqrt(
        pose.orientation.x**2
        + pose.orientation.y**2
        + pose.orientation.z**2
        + pose.orientation.w**2
    )
    if quaternion_norm < 1e-9:
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
    else:
        pose.orientation.x /= quaternion_norm
        pose.orientation.y /= quaternion_norm
        pose.orientation.z /= quaternion_norm
        pose.orientation.w /= quaternion_norm
    return pose


class GazeboPoseSync(Node):
    """Throttle odometry into Gazebo's SetEntityPose service."""

    def __init__(self):
        super().__init__("gazebo_pose_sync")
        self.entity_name = self.declare_parameter("entity_name", "go2_visual").value
        self.body_pose_topic = self.declare_parameter(
            "body_pose_topic", "/quad_0/body_pose"
        ).value
        self.service_name = self.declare_parameter(
            "service_name", "/world/garage_visual/set_pose"
        ).value
        update_rate = float(self.declare_parameter("update_rate", 30.0).value)
        self.z_offset = float(self.declare_parameter("z_offset", 0.0).value)
        self.log_distance = float(self.declare_parameter("log_distance", 5.0).value)

        if not self.entity_name:
            raise ValueError("entity_name must not be empty")
        if not self.body_pose_topic or not self.service_name:
            raise ValueError("body_pose_topic and service_name must not be empty")
        if not math.isfinite(update_rate) or update_rate <= 0.0:
            raise ValueError("update_rate must be finite and positive")
        if not math.isfinite(self.z_offset):
            raise ValueError("z_offset must be finite")
        if not math.isfinite(self.log_distance) or self.log_distance < 0.0:
            raise ValueError("log_distance must be finite and non-negative")

        self.latest_pose = None
        self.pending_future = None
        self.synced_once = False
        self.last_logged_position = None
        self.service_wait_logged = False

        self.pose_subscription = self.create_subscription(
            Odometry,
            self.body_pose_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.pose_client = self.create_client(SetEntityPose, self.service_name)
        self.timer = self.create_timer(1.0 / update_rate, self.update_pose)
        self.get_logger().info(
            f"Waiting for {self.body_pose_topic} and {self.service_name} "
            f"to drive Gazebo model '{self.entity_name}'"
        )

    def odom_callback(self, message: Odometry):
        self.latest_pose = pose_for_gazebo(message.pose.pose, self.z_offset)

    def update_pose(self):
        if self.latest_pose is None:
            return
        if not self.pose_client.service_is_ready():
            if not self.service_wait_logged:
                self.get_logger().info(
                    f"Gazebo pose service is not ready yet: {self.service_name}"
                )
                self.service_wait_logged = True
            return
        self.service_wait_logged = False

        if self.pending_future is not None:
            if not self.pending_future.done():
                return
            try:
                response = self.pending_future.result()
            except Exception as error:  # rclpy surfaces transport errors here.
                self.get_logger().error(f"Gazebo pose request failed: {error}")
                self.pending_future = None
                return
            self.pending_future = None
            if response is None or not response.success:
                self.get_logger().error(
                    f"Gazebo rejected pose update for '{self.entity_name}'"
                )
                return

        pose = copy.deepcopy(self.latest_pose)
        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose
        self.pending_future = self.pose_client.call_async(request)

        position = pose.position
        should_log = not self.synced_once
        if self.last_logged_position is not None and self.log_distance > 0.0:
            dx = position.x - self.last_logged_position[0]
            dy = position.y - self.last_logged_position[1]
            dz = position.z - self.last_logged_position[2]
            should_log = should_log or math.sqrt(dx * dx + dy * dy + dz * dz) >= self.log_distance
        if should_log:
            prefix = "First" if not self.synced_once else "Advanced"
            self.get_logger().info(
                f"{prefix} Gazebo pose: "
                f"({position.x:.2f}, {position.y:.2f}, {position.z:.2f})"
            )
            self.last_logged_position = (position.x, position.y, position.z)
        self.synced_once = True


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch may forward a second SIGINT while Python is already
        # unwinding the executor. Keep demo shutdown free of a traceback.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
