"""health_monitor + task_manager + stack_lifecycle_manager (same namespace as heartbeat)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _slam_params(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context).strip().strip("/") or "robot2"
    mapper = LaunchConfiguration("mapper_params_file").perform(context)
    localize = LaunchConfiguration("localization_params_file").perform(context)
    initial = LaunchConfiguration("initial_slam_mode").perform(context).strip() or "inactive"
    map_file = LaunchConfiguration("map_file").perform(context).strip()
    use_sim = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in (
        "true",
        "1",
        "yes",
    )
    return [
        {
            "use_sim_time": use_sim,
            "mapper_params_file": mapper,
            "localization_params_file": localize,
            "map_file": map_file,
            "map_frame": "map",
            "odom_frame": f"{ns}/odom",
            "base_frame": f"{ns}/base_footprint",
            "scan_topic": f"/{ns}/scan_2d",
            "mapping_map_topic": f"/{ns}/mapping",
            "localization_map_topic": f"/{ns}/map",
            "slam_child_namespace": f"/{ns}/slam_bringup",
            "initial_slam_mode": initial,
            "tracked_lifecycle_nodes": ["heartbeat", "lifecycle_manager_navigation"],
        }
    ]


def _launch_setup(context, *args, **kwargs):
    slam_params = _slam_params(context)[0]
    return [
        Node(
            package="manager",
            executable="stack_lifecycle_manager_node",
            name="stack_lifecycle_manager",
            output="screen",
            parameters=[slam_params],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="robot2",
                description="PushRosNamespace for this robot (match heartbeat).",
            ),
            DeclareLaunchArgument(
                "localization_pose_topic",
                default_value="amcl_pose",
                description="Empty string disables pose-based ready transition.",
            ),
            DeclareLaunchArgument(
                "require_nav2",
                default_value="false",
                description=(
                    "When 'true', bt_navigator must be running before health_monitor "
                    "advances to the localizing phase."
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "mapper_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manager"), "config", "mapper_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manager"), "config", "localization_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="",
                description="Map yaml for localize slam mode (initial_slam_mode:=localize).",
            ),
            DeclareLaunchArgument(
                "initial_slam_mode",
                default_value="inactive",
                description="slam mode at startup: inactive|mapping|localize",
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(LaunchConfiguration("namespace")),
                    Node(
                        package="manager",
                        executable="health_monitor_node",
                        name="health_monitor",
                        output="screen",
                        condition=UnlessCondition(LaunchConfiguration("require_nav2")),
                        parameters=[
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                                "required_nodes": ["heartbeat"],
                            }
                        ],
                    ),
                    Node(
                        package="manager",
                        executable="health_monitor_node",
                        name="health_monitor",
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("require_nav2")),
                        parameters=[
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                                "required_nodes": ["heartbeat", "bt_navigator"],
                            }
                        ],
                    ),
                    Node(
                        package="manager",
                        executable="task_manager_node",
                        name="task_manager",
                        output="screen",
                    ),
                    OpaqueFunction(function=_launch_setup),
                ]
            ),
        ]
    )
