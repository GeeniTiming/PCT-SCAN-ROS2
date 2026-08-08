from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_FILE = Path(__file__).parents[1] / "worlds" / "garage_visual.sdf"


def test_visual_world_has_pose_command_consumers():
    root = ET.parse(WORLD_FILE).getroot()
    plugins = {
        plugin.attrib.get("name", "")
        for plugin in root.findall("./world/plugin")
    }

    assert "ignition::gazebo::systems::Physics" in plugins
    assert "ignition::gazebo::systems::UserCommands" in plugins
    assert "ignition::gazebo::systems::SceneBroadcaster" in plugins


def test_visual_world_does_not_load_robot_control():
    world_text = WORLD_FILE.read_text(encoding="utf-8")

    assert "gz_ros2_control" not in world_text
    assert "ignition_ros2_control" not in world_text
