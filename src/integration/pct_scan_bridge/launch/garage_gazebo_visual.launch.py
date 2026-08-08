"""Render Garage and a ROS-driven visual Go2 in Gazebo Fortress."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_root = os.environ.get("PCT_SCAN_ROS2_ROOT", "")
    if not project_root:
        raise RuntimeError(
            "PCT_SCAN_ROS2_ROOT is not set. Start this demo through "
            "scripts/launch_pct_scan_gazebo.sh."
        )

    garage_resource_root = os.path.join(project_root, "assets", "garage", "upstream")
    garage_model = os.path.join(garage_resource_root, "garage", "model.sdf")
    if not os.path.isfile(garage_model):
        raise RuntimeError(f"Garage Gazebo model is missing: {garage_model}")

    bridge_share = get_package_share_directory("pct_scan_bridge")
    go2_share = get_package_share_directory("go2_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    resource_roots = os.pathsep.join(
        [garage_resource_root, os.path.dirname(go2_share)]
    )
    world = os.path.join(bridge_share, "worlds", "garage_visual.sdf")
    robot_xacro = os.path.join(go2_share, "xacro", "robot.xacro")
    world_name = "garage_visual"
    pose_service = f"/world/{world_name}/set_pose"

    declarations = [
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("body_pose_topic", default_value="/quad_0/body_pose"),
        DeclareLaunchArgument("entity_name", default_value="go2_visual"),
        DeclareLaunchArgument("init_x", default_value="16.760008"),
        DeclareLaunchArgument("init_y", default_value="14.617265"),
        DeclareLaunchArgument("init_z", default_value="0.489722"),
        DeclareLaunchArgument("pose_update_rate", default_value="30.0"),
    ]

    common_resource_path = [
        resource_roots,
        os.pathsep,
        EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
    ]
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-s -r -v 3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_go2_visual",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        robot_xacro,
                        " use_gazebo:=false visual_only:=true",
                    ]
                )
            }
        ],
        arguments=[
            "-world",
            world_name,
            "-param",
            "robot_description",
            "-name",
            LaunchConfiguration("entity_name"),
            "-allow_renaming",
            "false",
            "-x",
            LaunchConfiguration("init_x"),
            "-y",
            LaunchConfiguration("init_y"),
            "-z",
            LaunchConfiguration("init_z"),
        ],
    )
    service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="garage_pose_service_bridge",
        output="screen",
        arguments=[f"{pose_service}@ros_gz_interfaces/srv/SetEntityPose"],
    )
    pose_sync = Node(
        package="pct_scan_bridge",
        executable="gazebo_pose_sync",
        name="gazebo_pose_sync",
        output="screen",
        parameters=[
            {
                "entity_name": LaunchConfiguration("entity_name"),
                "body_pose_topic": LaunchConfiguration("body_pose_topic"),
                "service_name": pose_service,
                "update_rate": ParameterValue(
                    LaunchConfiguration("pose_update_rate"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            *declarations,
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", common_resource_path
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    resource_roots,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            gazebo_gui,
            gazebo_headless,
            service_bridge,
            spawn,
            pose_sync,
        ]
    )
