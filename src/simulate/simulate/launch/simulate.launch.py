"""Gazebo 无头仿真主入口（单文件）。与 openDelivery/params/launch/simulate/simulate.launch.py 保持同步.

Only the **odometry** TF tree is published here: ``robotN/odom`` → ``robotN/base_footprint`` → ``robotN/base_link``.
The **map** frame and ``map``→``odom`` are owned by **SLAM** (or other localization); do not add ``map`` in this launch.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _as_bool(text: str) -> bool:
    return (text or "").strip().lower() in ("true", "1", "yes", "on")


def _resolve_world_path(share: str, world_arg: str) -> str:
    """Allow both absolute world path and worlds/<name> shorthand."""
    world_arg = (world_arg or "").strip()
    if world_arg.startswith("/"):
        return world_arg
    return os.path.join(share, "worlds", world_arg or "drawn_model.world")


def _run_xacro_robot_description(namespace: str) -> str:
    """Namespace for ROS plugins; frame_prefix for TF link names (empty if global)."""
    ns = (namespace or "").strip()
    fp = "" if ns in ("/", "") else f"{ns}/"
    share = get_package_share_directory("simulate")
    urdf = os.path.join(share, "urdf", "simple_2d_robot.urdf.xacro")
    out = subprocess.check_output(
        [
            "xacro",
            urdf,
            f"robot_namespace:={ns}",
            f"frame_prefix:={fp}",
        ],
        stderr=subprocess.STDOUT,
    )
    return out.decode("utf-8")


def _spawn_pose_from_slot(slot_text: str):
    slots = {
        "0": (0.0, 0.0, 0.05, 0.0),
        "1": (-10.0, 10.0, 0.05, 0.0),
        "2": (10.0, 10.0, 0.05, 0.0),
        "3": (-10.0, -10.0, 0.05, 0.0),
        "4": (10.0, -10.0, 0.05, 0.0),
    }
    slot = (slot_text or "0").strip()
    if slot not in slots:
        raise RuntimeError(
            f"invalid spawn_slot={slot!r}; expected one of 0, 1, 2, 3, 4"
        )
    return slots[slot]


def _resolve_spawn_pose(context):
    sx = LaunchConfiguration("spawn_x").perform(context).strip()
    sy = LaunchConfiguration("spawn_y").perform(context).strip()
    sz = LaunchConfiguration("spawn_z").perform(context).strip()
    syaw = LaunchConfiguration("spawn_yaw").perform(context).strip()
    if sx or sy or sz or syaw:
        try:
            return (
                float(sx or "0.0"),
                float(sy or "0.0"),
                float(sz or "0.05"),
                float(syaw or "0.0"),
            )
        except ValueError as exc:
            raise RuntimeError(
                "spawn_x/spawn_y/spawn_z/spawn_yaw must be numeric when set"
            ) from exc
    return _spawn_pose_from_slot(LaunchConfiguration("spawn_slot").perform(context))


def launch_setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration("namespace").perform(context).strip()
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or "sim_robot"
    entity_name = ns or robot_name
    use_sim = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    start_gz = _as_bool(LaunchConfiguration("start_gazebo").perform(context))
    start_xvfb = _as_bool(LaunchConfiguration("start_xvfb").perform(context))
    spawn_robot = _as_bool(LaunchConfiguration("spawn_robot").perform(context))
    xvfb_display = LaunchConfiguration("xvfb_display").perform(context).strip() or ":99"
    world = LaunchConfiguration("world").perform(context)
    share = get_package_share_directory("simulate")
    model_dir = os.path.join(share, "model")
    world_path = _resolve_world_path(share, world)

    actions = []

    if start_gz:
        gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
        merged_model_path = (
            model_dir
            if not gazebo_model_path
            else f"{model_dir}{os.pathsep}{gazebo_model_path}"
        )
        actions.append(
            SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=merged_model_path)
        )
        actions.append(
            SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="minimal")
        )
        actions.append(
            SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
        )
        if start_xvfb:
            actions.append(SetEnvironmentVariable(name="DISPLAY", value=xvfb_display))
            actions.append(
                LogInfo(
                    msg=(
                        f"[simulate] starting Xvfb on {xvfb_display} for headless camera rendering"
                    )
                )
            )
            actions.append(
                ExecuteProcess(
                    cmd=[
                        "Xvfb",
                        xvfb_display,
                        "-screen",
                        "0",
                        "1280x720x24",
                        "-nolisten",
                        "tcp",
                    ],
                    output="screen",
                )
            )
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "gui": "false",
                "verbose": "false",
                "world": world_path,
            }.items(),
        )
        if start_xvfb:
            actions.append(TimerAction(period=0.8, actions=[gazebo_launch]))
        else:
            actions.append(gazebo_launch)

    start_gzweb = _as_bool(LaunchConfiguration("start_gzweb").perform(context))
    if start_gzweb:
        gzweb_root_arg = LaunchConfiguration("gzweb_root").perform(context).strip()
        if not gzweb_root_arg:
            gzweb_root_arg = os.environ.get("GZWEB_ROOT", "").strip()
        if not gzweb_root_arg:
            vendored_root = os.path.join(share, "third_party", "gzweb")
            if os.path.isfile(os.path.join(vendored_root, "package.json")):
                gzweb_root_arg = vendored_root
        port = LaunchConfiguration("gzweb_port").perform(context).strip() or "8080"
        raw_delay = LaunchConfiguration("gzweb_start_delay").perform(context).strip()
        try:
            delay_sec = float(raw_delay if raw_delay else "8.0")
        except ValueError:
            delay_sec = 8.0
        delay_sec = max(0.0, delay_sec)
        script = os.path.join(share, "scripts", "start_gzweb.sh")
        pkg_json = (
            os.path.join(gzweb_root_arg, "package.json") if gzweb_root_arg else ""
        )
        if gzweb_root_arg and os.path.isfile(pkg_json):
            actions.append(
                LogInfo(
                    msg=(
                        f"[simulate] GzWeb will start in {delay_sec}s "
                        f"(root={gzweb_root_arg}, port={port})"
                    )
                )
            )
            gzweb_proc = ExecuteProcess(
                cmd=["bash", script, gzweb_root_arg, port],
                output="screen",
            )
            actions.append(
                TimerAction(period=delay_sec, actions=[gzweb_proc])
            )
        else:
            actions.append(
                LogInfo(
                    msg=(
                        "[simulate] start_gzweb:=true but gzweb_root empty or not a GzWeb "
                        "directory (missing package.json). Set gzweb_root:=/path/to/gzweb, "
                        "or export GZWEB_ROOT, or download源码到 src/simulate/gzweb then "
                        "colcon build --packages-select simulate."
                    )
                )
            )

    if spawn_robot:
        spawn_x, spawn_y, spawn_z, spawn_yaw = _resolve_spawn_pose(context)
        try:
            robot_desc = _run_xacro_robot_description(ns)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(e.output.decode("utf-8", errors="replace")) from e

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim, "robot_description": robot_desc}],
        )
        spawn_robot_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                entity_name,
                "-topic",
                "robot_description",
                "-timeout",
                "180",
                "-spawn_service_timeout",
                "180",
                "-x",
                str(spawn_x),
                "-y",
                str(spawn_y),
                "-z",
                str(spawn_z),
                "-Y",
                str(spawn_yaw),
            ],
            output="screen",
        )
        group_children = [PushRosNamespace(ns), robot_state_publisher, spawn_robot_node]
        actions.append(GroupAction(actions=group_children))
        actions.append(
            LogInfo(
                msg=(
                    f"[simulate] spawn robot entity={entity_name} namespace={ns or '/'} "
                    f"at x={spawn_x:.2f}, y={spawn_y:.2f}, z={spawn_z:.2f}, yaw={spawn_yaw:.2f}"
                )
            )
        )
    else:
        actions.append(
            LogInfo(
                msg="[simulate] spawn_robot:=false -> world/map only (no robot entity spawned)"
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "robot_name",
                default_value="sim_robot",
                description="When namespace is not set, also selects default namespace.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value=LaunchConfiguration("robot_name"),
                description="ROS namespace + TF prefix (e.g. robot2 → robot2/odom, robot2/base_link).",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="If false, skip starting Gazebo (use after a first full launch is running).",
            ),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="true",
                description="If false, only load the Gazebo world/map once and skip robot spawning.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="drawn_model.world",
                description="Gazebo world file name under simulate/worlds, or absolute path.",
            ),
            DeclareLaunchArgument(
                "spawn_slot",
                default_value="0",
                description="Predefined spawn slot: 0=center, 1=top-left, 2=top-right, 3=bottom-left, 4=bottom-right.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="",
                description="Optional explicit spawn X in world coordinates; overrides spawn_slot when set.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="",
                description="Optional explicit spawn Y in world coordinates; overrides spawn_slot when set.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="",
                description="Optional explicit spawn Z in world coordinates; overrides spawn_slot when set.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="",
                description="Optional explicit spawn yaw (rad); overrides spawn_slot when set.",
            ),
            DeclareLaunchArgument(
                "start_xvfb",
                default_value="true",
                description="Start Xvfb in headless environments so Gazebo camera sensors can render.",
            ),
            DeclareLaunchArgument(
                "xvfb_display",
                default_value=":99",
                description="DISPLAY value used by Xvfb/gzserver when start_xvfb is true.",
            ),
            DeclareLaunchArgument(
                "start_gzweb",
                default_value="false",
                description=(
                    "If true, start OSRF GzWeb (npm) after a delay; uses gzweb_root, or "
                    "GZWEB_ROOT, or installed vendored source share/simulate/third_party/gzweb."
                ),
            ),
            DeclareLaunchArgument(
                "gzweb_root",
                default_value="",
                description="Absolute path to GzWeb repository root (overrides GZWEB_ROOT env if set).",
            ),
            DeclareLaunchArgument(
                "gzweb_port",
                default_value="8080",
                description="HTTP port passed to GzWeb (PORT env).",
            ),
            DeclareLaunchArgument(
                "gzweb_start_delay",
                default_value="8.0",
                description="Seconds to wait after launch before starting GzWeb (lets gzserver come up).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
