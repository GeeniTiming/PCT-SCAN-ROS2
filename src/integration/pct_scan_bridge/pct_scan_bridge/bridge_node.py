"""ROS 2 bridge between PCT global planning and SCAN local planning."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from visualization_msgs.msg import (
    InteractiveMarkerControl, InteractiveMarkerFeedback, Marker,
)

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
        # The parent builds the interactive markers during construction.
        self.goal_yaw = 0.0
        self.goal_heading_pub = None
        # PCTPlanner constructs the class through this module-level symbol.
        # Override it only inside this bridge process; the dependency checkout
        # and its installed files stay untouched.
        upstream_planner_node.TomogramPlanner = DynamicLayerTomogramPlanner
        super().__init__()

        self.goal_heading_pub = self.create_publisher(
            PoseStamped,
            "/pct_planner/goal_pose_preview",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.publish_goal_heading_preview()
        self.get_logger().info(
            "Goal heading editor ready: rotate the Goal ring to save map yaw. "
            "Heading is carried by /pct_path; SCAN/controller handle terminal alignment."
        )

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

    def make_interactive_marker(self, name, description, waypoint, color):
        """Keep position handles separate from the Goal's saved heading."""
        marker = super().make_interactive_marker(name, description, waypoint, color)
        marker.description = f"{description} (position)"
        marker.controls = [
            control for control in marker.controls
            if control.name not in ("move_x", "move_y")
        ]
        for control in marker.controls:
            if control.name == f"{name}_drag":
                # Drag on the map XY plane, independent of the viewing angle.
                plane = self.make_xy_plane_control()
                control.orientation = plane.orientation
                control.orientation_mode = plane.orientation_mode
                control.description = f"Drag {description} position on map XY"
            elif control.name == "move_z":
                control.description = "Adjust height only (map Z)"
        if name == "goal":
            marker.description = "Goal (heading preview)"
            marker.pose.orientation = self.goal_orientation()
            # The sphere already moves in XY. Replace the redundant XY disc
            # so that the new ring has one unambiguous purpose: editing yaw.
            marker.controls = [
                control for control in marker.controls if control.name != "move_xy"
            ]

            visual = InteractiveMarkerControl()
            visual.name = "goal_heading_arrow"
            visual.orientation.w = 1.0
            visual.interaction_mode = InteractiveMarkerControl.NONE
            visual.orientation_mode = InteractiveMarkerControl.INHERIT
            visual.always_visible = True
            arrow = Marker()
            arrow.type = Marker.ARROW
            arrow.pose.orientation.w = 1.0
            arrow.pose.position.z = 0.5
            arrow.scale.x = 1.0
            arrow.scale.y = 0.08
            arrow.scale.z = 0.16
            arrow.color.r = 1.0
            arrow.color.g = 0.65
            arrow.color.b = 0.05
            arrow.color.a = 1.0
            visual.markers.append(arrow)
            marker.controls.append(visual)

            rotate = InteractiveMarkerControl()
            rotate.name = "rotate_goal_yaw"
            # ROTATE_AXIS uses control X. Rotate it onto positive map Z.
            rotate.orientation.w = math.sqrt(0.5)
            rotate.orientation.y = -math.sqrt(0.5)
            rotate.orientation_mode = InteractiveMarkerControl.FIXED
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            rotate.always_visible = True
            rotate.description = "Rotate Goal heading on map XY (preview only)"
            marker.controls.append(rotate)
        return marker

    @staticmethod
    def yaw_from_orientation(orientation):
        values = (orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.hypot(*values)
        if not math.isfinite(norm) or norm < 1e-12:
            return None
        x, y, z, w = (value / norm for value in values)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def goal_orientation(self):
        return Quaternion(z=math.sin(self.goal_yaw / 2.0),
                          w=math.cos(self.goal_yaw / 2.0))

    def waypoint_world_pose(self, name, waypoint):
        xyz = self.planner_to_world(waypoint)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        pose.orientation = self.goal_orientation() if name == "goal" else Quaternion(w=1.0)
        return pose

    def update_interactive_marker_pose(self, name, waypoint):
        if self.marker_server is not None:
            self.marker_server.setPose(name, self.waypoint_world_pose(name, waypoint))
            self.marker_server.applyChanges()

    def publish_goal_heading_preview(self):
        if self.goal_heading_pub is None or self.goal_pose is None:
            return
        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = self.waypoint_world_pose("goal", self.goal_pose)
        self.goal_heading_pub.publish(message)

    def set_goal(self, x, y, z, plan=True, update_marker=True):
        # Preserve the saved yaw when the parent snaps/updates the position.
        success = super().set_goal(x, y, z, plan=False, update_marker=update_marker)
        if success:
            self.publish_goal_heading_preview()
            if plan:
                self.plan_current_waypoints()
        return success

    def interactive_marker_cb(self, feedback):
        if feedback.marker_name == "goal" and feedback.control_name == "rotate_goal_yaw":
            if feedback.event_type not in (
                InteractiveMarkerFeedback.POSE_UPDATE,
                InteractiveMarkerFeedback.MOUSE_UP,
            ):
                return
            yaw = self.yaw_from_orientation(feedback.pose.orientation)
            if yaw is None:
                self.get_logger().warn("Ignoring Goal heading with invalid quaternion")
                self.update_interactive_marker_pose("goal", self.goal_pose)
                return
            self.goal_yaw = yaw
            # Rotation must neither re-snap the position nor trigger planning,
            # even if plan_on_marker_release is enabled for translation.
            self.update_interactive_marker_pose("goal", self.goal_pose)
            self.publish_goal_heading_preview()
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                self.get_logger().info(
                    f"Goal heading saved: map yaw={math.degrees(yaw):.1f} deg "
                    "(preview only; select Plan route separately)"
                )
            return
        super().interactive_marker_cb(feedback)

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
            yaw = self.yaw_from_orientation(message.pose.orientation)
            if yaw is None:
                self.get_logger().warn("Ignoring goal pose with invalid quaternion")
                return
            previous_yaw = self.goal_yaw
            self.goal_yaw = yaw
            position = message.pose.position
            # Keep the compatibility input's existing plan-on-message behavior.
            if not self.set_goal(position.x, position.y, position.z):
                self.goal_yaw = previous_yaw
                if self.goal_pose is not None:
                    self.update_interactive_marker_pose("goal", self.goal_pose)

    def clicked_point_cb(self, message):
        if self.message_frame_matches(message, "clicked point"):
            super().clicked_point_cb(message)

    def build_path_message(self, traj_world):
        message = super().build_path_message(traj_world)
        if message.poses:
            message.poses[-1].pose.orientation = self.goal_orientation()
            self.get_logger().info(
                f"Path terminal heading: map yaw={math.degrees(self.goal_yaw):.1f} deg "
                "(terminal target heading)"
            )
        return message

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
