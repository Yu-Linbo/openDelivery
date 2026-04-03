from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _normalize_namespace(raw_namespace: str, robot_name: str) -> str:
    namespace = (raw_namespace or "").strip()
    if namespace in ("", "/"):
        namespace = (robot_name or "").strip()
    return namespace.strip("/")


def _frame_with_namespace(namespace: str, frame_name: str) -> str:
    frame = (frame_name or "").strip().lstrip("/")
    if not frame:
        return ""
    if not namespace:
        return frame
    return f"{namespace}/{frame}"


def _as_bool(text: str) -> bool:
    return (text or "").strip().lower() in ("true", "1", "yes", "on")


def _launch_setup(context, *_args, **_kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    map_file = LaunchConfiguration("map_file").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or "sim_robot"
    robot_ns = _normalize_namespace(
        LaunchConfiguration("namespace").perform(context), robot_name
    )
    node_ns = f"{robot_ns}/slam_bringup"
    map_frame = LaunchConfiguration("map_frame").perform(context).strip().lstrip("/") or "map"
    odom_frame = _frame_with_namespace(
        robot_ns, LaunchConfiguration("odom_frame").perform(context)
    )
    base_frame = _frame_with_namespace(
        robot_ns, LaunchConfiguration("base_frame").perform(context)
    )
    scan_arg = LaunchConfiguration("scan_topic").perform(context).strip()
    if scan_arg.startswith("/"):
        scan_topic = scan_arg
    else:
        scan_topic = f"/{robot_ns}/{scan_arg.lstrip('/') or 'scan_2d'}"
    map_name_arg = LaunchConfiguration("map_name").perform(context).strip()
    # Align with nav_bringup grid_mode=localize: publish /<robot>/map for Nav2 costmap remap.
    map_name = map_name_arg if map_name_arg else f"/{robot_ns}/map"

    slam_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="localization",
        namespace=node_ns,
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "mode": "localization",
                "map_file_name": map_file,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "scan_topic": scan_topic,
                "map_name": map_name,
            },
        ],
    )
    return [slam_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="sim_robot"),
            DeclareLaunchArgument(
                "namespace",
                default_value=LaunchConfiguration("robot_name"),
                description="Robot prefix for TF/sensors; node FQN /<namespace>/slam_bringup/localization.",
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan_2d",
                description="LaserScan: basename → /<namespace>/scan_2d, or absolute path.",
            ),
            DeclareLaunchArgument(
                "map_name",
                default_value="",
                description="OccupancyGrid topic (absolute). Empty → /<namespace>/map (robot prefix).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("slam_bringup"), "config", "localization_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="",
                description="Absolute path to map yaml for localization mode.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
