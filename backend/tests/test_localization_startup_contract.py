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


def test_sim_bringup_accepts_gazebo_plugin_topics_as_entity_readiness():
    source = (
        ROOT / "src" / "system" / "system" / "scripts" / "sim_bringup.sh"
    ).read_text(encoding="utf-8")
    assert 'grep -qx "/${RID}/odom"' in source
    assert 'grep -qx "/${RID}/scan_2d"' in source
    assert "ROS2CLI_DISABLE_DAEMON:=1" in source
    assert "timeout --signal=TERM --kill-after=1s 4s" in source
    assert 'ros2 lifecycle get "${HB}"' in source


def test_simulation_localization_defaults_to_gazebo_ground_truth():
    sim_bringup = (
        ROOT / "src" / "system" / "system" / "scripts" / "sim_bringup.sh"
    ).read_text(encoding="utf-8")
    manager_launch = (
        ROOT / "src" / "system" / "manager" / "launch" / "manager.launch.py"
    ).read_text(encoding="utf-8")
    heartbeat_launch = (
        ROOT / "params" / "launch" / "system" / "heartbeat.launch.py"
    ).read_text(encoding="utf-8")

    assert 'LOCALIZATION_METHOD="gazebo_ground_truth"' in sim_bringup
    assert 'or "gazebo_ground_truth"' in manager_launch
    assert 'default_value="gazebo_ground_truth"' in manager_launch
    assert 'default_value="gazebo_ground_truth"' in heartbeat_launch


def test_map_server_startup_waits_for_lifecycle_discovery():
    source = (
        ROOT / "src" / "system" / "manager" / "src"
        / "stack_lifecycle_manager_node.cpp"
    ).read_text(encoding="utf-8")
    start_map_server = source[
        source.index("bool StackLifecycleManagerNode::start_map_server"):
        source.index("bool StackLifecycleManagerNode::ensure_static_map_loaded")
    ]

    assert "ensure_lifecycle_active(fqn, &local_err)" in start_map_server
    assert 'call_lifecycle_transition(fqn, "configure"' not in start_map_server
    assert "std::chrono::seconds(15)" in source


def test_ready_disarms_startup_relocalization_before_later_floor_switches():
    source = (
        ROOT / "src" / "slam" / "relocalization" / "src"
        / "relocalization_node.cpp"
    ).read_text(encoding="utf-8")
    ready_branch = source[
        source.index('if (next_status == "ready")'):
        source.index('else if (next_status == "localizing")')
    ]

    assert "auto_relocalize_pending_ = false" in ready_branch
    assert "auto_relocalize_done_ = true" in ready_branch

    assert "pose-first relocalization cannot switch maps" in source
