"""Exercise both odom APIs through the real ROS node, without robot topics.

Run in an isolated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1. Only the expensive
map backend is stubbed; parameter declarations, callbacks and gates are real.
"""
import math
import pytest
import rclpy
from nav_msgs.msg import Odometry
from pct_scan_bridge import bridge_node


class EmptyMap:
    def __init__(self, *args):
        pass

    def loadTomogram(self, *args):
        pass


@pytest.fixture(params=["odom_auto_start", "auto_start_from_odom"])
def bridge(request, monkeypatch):
    monkeypatch.setattr(bridge_node, "DynamicLayerTomogramPlanner", EmptyMap)
    rclpy.init(args=["--ros-args", "-p", request.param + ":=true",
                     "-p", "scene_name:=custom", "-p", "frame_id:=map",
                     "-p", "rsg_root:=/tmp", "-p", "publish_tomogram:=false",
                     "-p", "enable_interactive_markers:=false",
                     "-p", "restrict_endpoints:=false"])
    node = bridge_node.PCTScanBridge()
    try:
        yield node, request.param == "odom_auto_start"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_alias_preserves_fresh_start_gate(bridge, monkeypatch):
    node, field_api = bridge
    planned = []
    starts = []
    monkeypatch.setattr(bridge_node.upstream_planner_node.PCTPlanner,
                        "plan_current_waypoints", lambda self: planned.append(True))
    monkeypatch.setattr(node, "set_start", lambda *xyz, **kw: starts.append(xyz) or True)
    assert node.auto_start_from_odom
    # Construction must not register duplicate parameters or duplicate odom subscriptions.
    assert len([s for s in node.subscriptions if s.topic_name == "/lightning/odom"]) == 1
    node.plan_current_waypoints()
    assert not planned

    msg = Odometry()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 1.0
    callback = node.odom_start_cb if field_api else node.odom_callback
    callback(msg)
    assert not planned  # Following a marker never triggers A*.
    node.plan_current_waypoints()
    assert len(planned) == 1
    assert starts[-1] == (1.0, 0.0, 0.0)

    for frame, x in [("odom", 2.0), ("map", math.nan)]:
        msg.header.frame_id = frame
        msg.pose.pose.position.x = x
        callback(msg)
        node.plan_current_waypoints()
        assert len(planned) == 1

    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 3.0
    callback(msg)
    node.odom_start_policy._received_at_ns -= 2_000_000_000
    node.plan_current_waypoints()
    assert len(planned) == 1
