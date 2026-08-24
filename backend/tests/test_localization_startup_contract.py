from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_web_start_rebuilds_all_custom_message_consumers_on_abi_change():
    source = (ROOT / "start_web_stack.sh").read_text(encoding="utf-8")
    assert ".open_delivery_custom_msgs_abi.sha256" in source
    assert "--packages-above-and-dependencies custom_msgs_srvs" in source
    assert "colcon build --packages-select custom_msgs_srvs" not in source
    assert "console_direct+ || true" not in source


def test_truth_localizer_lives_under_slam_and_world_publishes_model_states():
    package = ROOT / "src" / "slam" / "gazebo_ground_truth_localization"
    assert (package / "package.xml").is_file()
    assert (package / "scripts" / "gazebo_ground_truth_localizer.py").is_file()
    assert not (
        ROOT / "src" / "simulate" / "simulate" / "scripts"
        / "gazebo_ground_truth_localizer.py"
    ).exists()
    for name in ("drawn_model.world", "empty.world"):
        world = (
            ROOT / "src" / "simulate" / "simulate" / "worlds" / name
        ).read_text(encoding="utf-8")
        assert 'filename="libgazebo_ros_state.so"' in world
        assert "<namespace>/gazebo</namespace>" in world


def test_pose_only_web_command_carries_selected_map():
    source = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
    marker = 'type: "localize_nav_command"'
    command_block = source[source.index(marker): source.index(marker) + 500]
    assert "map_name: targetMapName()" in command_block
