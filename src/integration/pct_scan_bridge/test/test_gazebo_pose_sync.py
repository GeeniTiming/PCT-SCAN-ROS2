import math

from geometry_msgs.msg import Pose

from pct_scan_bridge.gazebo_pose_sync import pose_for_gazebo


def test_pose_for_gazebo_copies_offsets_and_normalizes():
    source = Pose()
    source.position.x = 1.0
    source.position.y = 2.0
    source.position.z = 3.0
    source.orientation.z = 2.0
    source.orientation.w = 2.0

    result = pose_for_gazebo(source, 0.25)

    assert result is not source
    assert math.isclose(result.position.z, 3.25)
    assert math.isclose(source.position.z, 3.0)
    assert math.isclose(result.orientation.z, math.sqrt(0.5))
    assert math.isclose(result.orientation.w, math.sqrt(0.5))


def test_pose_for_gazebo_repairs_zero_quaternion():
    result = pose_for_gazebo(Pose(), 0.0)
    assert result.orientation.x == 0.0
    assert result.orientation.y == 0.0
    assert result.orientation.z == 0.0
    assert result.orientation.w == 1.0
