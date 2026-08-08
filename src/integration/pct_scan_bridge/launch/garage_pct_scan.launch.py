"""Launch PCT global planning and SCAN local planning for Garage."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = os.environ.get("PCT_SCAN_ROS2_ROOT", "")
    if not project_root:
        raise RuntimeError(
            "PCT_SCAN_ROS2_ROOT is not set. Start this demo through "
            "scripts/launch_pct_scan_garage.sh."
        )
    default_tomogram = (
        os.path.join(
            project_root,
            "assets",
            "garage",
            "tomogram",
            "garage_pct_0p10.pickle",
        )
    )
    default_pcd = (
        os.path.join(
            project_root,
            "assets",
            "garage",
            "pcd",
            "garage_scan_0p20.pcd",
        )
    )

    bridge_share = get_package_share_directory("pct_scan_bridge")
    scan_share = get_package_share_directory("scan_planner")

    declarations = [
        DeclareLaunchArgument("tomogram_path", default_value=default_tomogram),
        DeclareLaunchArgument("pcd_map_file", default_value=default_pcd),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gpu", default_value="false"),
        DeclareLaunchArgument("controller_mode", default_value="open_loop"),
        DeclareLaunchArgument("plan_on_startup", default_value="false"),
        DeclareLaunchArgument("publish_tomogram", default_value="true"),
        DeclareLaunchArgument("enable_interactive_markers", default_value="true"),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("gazebo_headless", default_value="false"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "garage_bridge.launch.py")
        ),
        launch_arguments={
            "tomogram_path": LaunchConfiguration("tomogram_path"),
            "rsg_root": os.path.join(project_root, "assets", "garage"),
            "path_topic": "/pct_path",
            "publish_tomogram": LaunchConfiguration("publish_tomogram"),
            "plan_on_startup": LaunchConfiguration("plan_on_startup"),
            "enable_interactive_markers": LaunchConfiguration(
                "enable_interactive_markers"
            ),
            "plan_on_marker_release": "false",
        }.items(),
    )

    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "run.launch.py")
        ),
        launch_arguments={
            "is_real_world": "false",
            "navi_mode": "3",
            "sensor_type": "lidar",
            "controller_mode": LaunchConfiguration("controller_mode"),
            "initial_path_topic": "/pct_path",
            "reference_path_min_distance": "0.2",
            "reference_path_simplify_tolerance": "0.1",
            "planning_horizon": "1.0",
            "use_gpu": LaunchConfiguration("use_gpu"),
            "use_pcd_map": "true",
            "pcd_map_file": LaunchConfiguration("pcd_map_file"),
            "map_size_x": "180.0",
            "map_size_y": "240.0",
            "map_size_z": "25.0",
            "init_x": "16.760008",
            "init_y": "14.617265",
            "init_z": "0.489722",
            "use_sim_time": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(scan_share, "rviz", "default.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "garage_gazebo_visual.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("gazebo_headless"),
            "body_pose_topic": "/quad_0/body_pose",
            "init_x": "16.760008",
            "init_y": "14.617265",
            "init_z": "0.489722",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    return LaunchDescription([*declarations, bridge, scan, rviz, gazebo])
