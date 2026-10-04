"""Check the installed launch templates match task-layer endpoint semantics."""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[4]


def load_module(path):
    spec = importlib.util.spec_from_file_location('nav_launch_check', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_materialized_params_keep_original_goal_and_matching_stop_window(tmp_path):
    module = load_module(ROOT / 'params/launch/nav_bringup/stack.launch.py')
    output = module._materialize_params(
        'robotcheck', str(ROOT / 'src/navigation/nav_bringup'), '/robotcheck/map',
        global_rolling_window='false', global_window_width='0', global_window_height='0',
        track_unknown_space='true', unknown_cost_value='255', allow_unknown='true',
        local_track_unknown_space='true', map_subscribe_transient_local='true')
    path = Path(output)
    try:
        config = yaml.safe_load(path.read_text())
    finally:
        path.unlink()
    controller = config['controller_server']['ros__parameters']
    assert config['planner_server']['ros__parameters']['GridBased']['tolerance'] == 0.0
    assert controller['goal_checker']['xy_goal_tolerance'] == controller['FollowPath']['xy_goal_tolerance'] == 0.10
    assert controller['goal_checker']['yaw_goal_tolerance'] == 0.15
    assert controller['goal_checker']['stateful'] is False
    assert controller['FollowPath']['debug_trajectory_details'] is False
    assert controller['FollowPath']['BaseObstacle.scale'] == 0.005
    assert 'BaseObstacle' in controller['FollowPath']['critics']
    assert 'ObstacleFootprint' in controller['FollowPath']['critics']
    assert controller['FollowPath']['ObstacleFootprint.scale'] > 0
    assert controller['FollowPath']['GoalAlign.forward_point_distance'] < controller['goal_checker']['xy_goal_tolerance']
    assert controller['progress_checker']['required_movement_radius'] < controller['goal_checker']['xy_goal_tolerance']
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_ = config['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint'] == global_['footprint']
    assert local['footprint_padding'] == global_['footprint_padding'] == 0.01
    assert 'robot_radius' not in local and 'robot_radius' not in global_
    assert local['plugins'][0] == 'static_layer'
    assert local['static_layer']['map_topic'] == global_['static_layer']['map_topic'] == '/robotcheck/map'
    assert local['static_layer']['map_subscribe_transient_local'] is True
    assert local['track_unknown_space'] is True
    recovery = config['recoveries_server']['ros__parameters']
    assert recovery['robot_base_frame'] == local['robot_base_frame']
    assert recovery['global_frame'] == local['global_frame']
    assert recovery['transform_tolerance'] == local['transform_tolerance']


def test_task_executor_and_nav2_share_navigate_action_namespace(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))
    from launch_ros.actions import Node
    from launch.substitutions import LaunchConfiguration, TextSubstitution
    module = load_module(ROOT / 'params/launch/nav_bringup/navigation_namespaced.launch.py')
    description = module.generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    navigator = next(node for node in nodes if 'bt_navigator' in str(node._Node__node_executable))
    # Inspect actual launch remappings, including substitutions, rather than grep.
    remaps = navigator._Node__remappings
    for source, target in remaps:
        if ''.join(part.text for part in source if isinstance(part, TextSubstitution)) == 'navigate_to_pose':
            rendered = ''.join('robotcheck' if isinstance(part, LaunchConfiguration) else part.text for part in target)
            assert rendered == '/robotcheck/navigation/navigate_to_pose'
            break
    else:
        raise AssertionError('NavigateToPose remap missing')


def test_stack_uses_tree_with_foxy_action_and_service_acknowledgement_timeouts(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    module = load_module(ROOT / 'params/launch/nav_bringup/stack.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: str(ROOT / 'src/navigation' / package))
    context = LaunchContext()
    context.launch_configurations.update(robot_name='robotcheck', grid_mode='localize',
                                        use_sim_time='false', autostart='true', robot_settings_file='')
    group = module._launch_setup(context)[0]
    include = next(action for action in group.get_sub_entities()
                   if isinstance(action, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    Path(arguments['params_file']).unlink()
    tree = ET.parse(arguments['default_bt_xml_filename'])
    # A YAML parameter from newer Nav2 releases does not change Foxy's
    # hardcoded 10 ms blackboard timeout: each ROS BT port must override it.
    for tag in ('ComputePathToPose', 'FollowPath', 'ClearEntireCostmap', 'Spin', 'Wait'):
        nodes = tree.findall('.//' + tag)
        assert nodes, tag
        assert all(int(node.get('server_timeout', '0')) >= 1000 for node in nodes), tag
    assert len(tree.findall('.//RecoveryNode')) == 3
    assert tree.find('.//RateController').get('hz') == '1.0'
