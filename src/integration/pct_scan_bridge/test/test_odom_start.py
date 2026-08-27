from types import SimpleNamespace

from pct_scan_bridge.odom_start import OdomStartPolicy


def make_odom(x=1.0, y=2.0, z=3.0, frame_id="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z)
            )
        ),
    )


def test_fresh_matching_odom_returns_offset_xyz():
    policy = OdomStartPolicy(0.5, "map", z_offset=-0.4)
    policy.update(make_odom(), 1_000_000_000)

    xyz, reason = policy.resolve(1_250_000_000)

    assert reason == "ok"
    assert xyz == (1.0, 2.0, 2.6)


def test_missing_stale_and_wrong_frame_are_rejected():
    policy = OdomStartPolicy(0.5, "map")
    assert policy.resolve(1_000_000_000) == (None, "odom_unavailable")

    policy.update(make_odom(), 1_000_000_000)
    xyz, reason = policy.resolve(1_600_000_000)
    assert xyz is None
    assert reason == "odom_stale_0.600s"

    policy.update(make_odom(frame_id="odom"), 2_000_000_000)
    assert policy.resolve(2_100_000_000) == (
        None,
        "odom_frame_odom_expected_map",
    )


def test_non_finite_position_is_rejected():
    policy = OdomStartPolicy(0.5, "map")
    policy.update(make_odom(x=float("nan")), 1_000_000_000)

    assert policy.resolve(1_100_000_000) == (
        None,
        "odom_position_non_finite",
    )
