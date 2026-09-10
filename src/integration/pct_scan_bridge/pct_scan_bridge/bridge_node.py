"""ROS 2 bridge between PCT global planning and SCAN local planning."""

from __future__ import annotations

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from pct_planner import planner_node as upstream_planner_node

from .dynamic_planner import DynamicLayerTomogramPlanner
from .odom_start import OdomStartPolicy
from .tomogram_visualization import build_full_cost_tomogram


BUILDING_START = np.array([5.0, 5.0, 0.0], dtype=np.float32)
BUILDING_GOAL = np.array([-6.0, -1.0, 14.0], dtype=np.float32)
SPIRAL_DEMO_GOAL = np.array([3.2, -22.2, 4.0], dtype=np.float32)


class PCTScanBridge(upstream_planner_node.PCTPlanner):
    """Reuse PCT's ROS interaction layer while owning integration fixes here."""

    def __init__(self):
        # PCTPlanner constructs the class through this module-level symbol.
        # Override it only inside this bridge process; the dependency checkout
        # and its installed files stay untouched.
        upstream_planner_node.TomogramPlanner = DynamicLayerTomogramPlanner
        super().__init__()

        self.declare_parameter("auto_start_from_odom", False)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.auto_start_from_odom = (
            self.get_parameter("auto_start_from_odom")
            .get_parameter_value()
            .bool_value
        )
        self.auto_start_from_odom = self.auto_start_from_odom or self.odom_auto_start
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.odom_start_policy = OdomStartPolicy(
            timeout_sec=(
                self.get_parameter("odom_timeout_sec")
                .get_parameter_value()
                .double_value
            ),
            expected_frame=self.frame_id,
            z_offset=(
                self.get_parameter("odom_start_z_offset")
                .get_parameter_value()
                .double_value
            ),
        )
        self.odom_sub = None
        if self.auto_start_from_odom and not self.odom_auto_start:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "Automatic PCT start enabled: each planning request snapshots "
                f"fresh odometry from {self.odom_topic} in frame {self.frame_id}"
            )
        elif not self.auto_start_from_odom:
            self.get_logger().info(
                "Automatic PCT start disabled; interactive/manual Start remains active"
            )

    def odom_start_cb(self, message):
        """Field-compatible marker following with frame/finite checks."""
        self.odom_callback(message)
        xyz, _ = self.odom_start_policy.resolve(self.get_clock().now().nanoseconds)
        if xyz is not None:
            super().odom_start_cb(message)

    def odom_callback(self, message):
        self.odom_start_policy.update(
            message, self.get_clock().now().nanoseconds
        )

    def refresh_start_from_odom(self):
        xyz, reason = self.odom_start_policy.resolve(
            self.get_clock().now().nanoseconds
        )
        if xyz is None:
            self.get_logger().error(
                "Planning rejected: cannot derive PCT start from odometry: " + reason
            )
            return False

        if not self.set_start(*xyz, plan=False, update_marker=True):
            self.get_logger().error(
                "Planning rejected: current odometry cannot be snapped to a "
                "traversable tomogram cell"
            )
            return False
        return True

    def message_frame_matches(self, message, input_name):
        actual_frame = str(message.header.frame_id).lstrip("/")
        expected_frame = self.frame_id.lstrip("/")
        if actual_frame != expected_frame:
            self.get_logger().error(
                f"Ignoring {input_name}: frame '{actual_frame or '<empty>'}' "
                f"does not match planner frame '{expected_frame}'"
            )
            return False
        return True

    def start_pose_cb(self, message):
        if self.message_frame_matches(message, "manual start"):
            super().start_pose_cb(message)

    def goal_pose_cb(self, message):
        if self.message_frame_matches(message, "goal"):
            super().goal_pose_cb(message)

    def clicked_point_cb(self, message):
        if self.message_frame_matches(message, "clicked point"):
            super().clicked_point_cb(message)

    def plan_current_waypoints(self):
        # During the base constructor this attribute does not exist yet.  This
        # also preserves the built-in plan_on_startup demos.
        if getattr(self, "auto_start_from_odom", False):
            if not self.refresh_start_from_odom():
                return
        super().plan_current_waypoints()

    def configure_scene(self, scene_name):
        # The upstream node calls configure_scene() after rclpy.Node has been
        # initialized but before it constructs and loads TomogramPlanner.  That
        # makes this the earliest safe point to expose bridge-owned A* tuning
        # as normal ROS parameters.
        self.declare_parameter("centerline_bias_enabled", True)
        self.declare_parameter("preferred_clearance", 0.5)
        self.declare_parameter("clearance_cost", 20.0)
        self.declare_parameter("astar_step_cost_weight", 1.0)
        self.declare_parameter("gateway_cost_delta", 8.0)
        self.declare_parameter("gateway_height_tolerance", 0.1)
        self.declare_parameter("tomogram_visual_min_cost", 0.0)
        self.declare_parameter("tomogram_visual_max_cost", 50.0)
        self.tomogram_visual_min_cost = (
            self.get_parameter("tomogram_visual_min_cost")
            .get_parameter_value()
            .double_value
        )
        self.tomogram_visual_max_cost = (
            self.get_parameter("tomogram_visual_max_cost")
            .get_parameter_value()
            .double_value
        )
        DynamicLayerTomogramPlanner.centerline_bias_enabled = (
            self.get_parameter("centerline_bias_enabled")
            .get_parameter_value()
            .bool_value
        )
        DynamicLayerTomogramPlanner.preferred_clearance = (
            self.get_parameter("preferred_clearance")
            .get_parameter_value()
            .double_value
        )
        DynamicLayerTomogramPlanner.clearance_cost = (
            self.get_parameter("clearance_cost").get_parameter_value().double_value
        )
        DynamicLayerTomogramPlanner.astar_step_cost_weight = (
            self.get_parameter("astar_step_cost_weight")
            .get_parameter_value()
            .double_value
        )
        DynamicLayerTomogramPlanner.gateway_cost_delta = (
            self.get_parameter("gateway_cost_delta")
            .get_parameter_value()
            .double_value
        )
        DynamicLayerTomogramPlanner.gateway_height_tolerance = (
            self.get_parameter("gateway_height_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(
            "PCT A* centerline bias: "
            f"enabled={DynamicLayerTomogramPlanner.centerline_bias_enabled}, "
            f"preferred_clearance="
            f"{DynamicLayerTomogramPlanner.preferred_clearance:.2f} m, "
            f"clearance_cost={DynamicLayerTomogramPlanner.clearance_cost:.1f}, "
            f"step_weight={DynamicLayerTomogramPlanner.astar_step_cost_weight:.1f}, "
            f"gateway_cost_delta={DynamicLayerTomogramPlanner.gateway_cost_delta:.1f}, "
            f"gateway_height_tolerance="
            f"{DynamicLayerTomogramPlanner.gateway_height_tolerance:.2f} m"
        )
        if scene_name.lower() == "building":
            self.tomo_file = "building2_9"
            self.start_pos = BUILDING_START.copy()
            self.end_pos = BUILDING_GOAL.copy()
        else:
            super().configure_scene(scene_name)
            if scene_name.lower() == "spiral":
                # The upstream endpoint returns to the ground after traversing
                # the entire 426 m spiral.  This shorter two-level route is a
                # practical, deterministic RViz demo while markers still allow
                # selecting any higher platform.
                self.end_pos = SPIRAL_DEMO_GOAL.copy()

        self.declare_parameter("use_scene_waypoint_overrides", False)
        values = {}
        for waypoint_name, waypoint in (
            ("start", self.start_pos),
            ("goal", self.end_pos),
        ):
            for axis, default in zip("xyz", waypoint):
                parameter_name = f"scene_{waypoint_name}_{axis}"
                self.declare_parameter(parameter_name, float(default))
                values[parameter_name] = (
                    self.get_parameter(parameter_name)
                    .get_parameter_value()
                    .double_value
                )
        if (
            self.get_parameter("use_scene_waypoint_overrides")
            .get_parameter_value()
            .bool_value
        ):
            self.start_pos = np.array(
                [values[f"scene_start_{axis}"] for axis in "xyz"],
                dtype=np.float32,
            )
            self.end_pos = np.array(
                [values[f"scene_goal_{axis}"] for axis in "xyz"],
                dtype=np.float32,
            )

    def publish_tomogram(self):
        """Publish the paper's complete 0--50 PCT travel-cost field."""
        planner_points, costs = build_full_cost_tomogram(
            np.where(self.planner.elev_g_valid, self.planner.elev_g, np.nan),
            self.planner.trav,
            self.planner.resolution,
            self.planner.center,
            self.planner.offset,
            self.planner.slice_dh,
            z_offset=self.tomogram_visual_z_offset,
            min_cost=self.tomogram_visual_min_cost,
            max_cost=self.tomogram_visual_max_cost,
        )
        world_points = self.planner_to_world(planner_points)
        cloud_points = np.column_stack((world_points, costs)).astype(np.float32)

        header = upstream_planner_node.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.tomogram_pub.publish(
            upstream_planner_node.pc2.create_cloud(
                header,
                upstream_planner_node.TOMOGRAM_POINT_FIELDS,
                cloud_points,
            )
        )
        self.get_logger().info(
            f"Published {cloud_points.shape[0]} full-cost tomogram cells on "
            f"{self.tomogram_topic}; visualization range="
            f"[{self.tomogram_visual_min_cost:.1f}, "
            f"{self.tomogram_visual_max_cost:.1f}]"
        )


def main(args=None):
    upstream_planner_node.rclpy.init(args=args)
    node = PCTScanBridge()
    try:
        upstream_planner_node.rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.try_shutdown() is not available in ROS 2 Foxy.
        if upstream_planner_node.rclpy.ok():
            upstream_planner_node.rclpy.shutdown()
