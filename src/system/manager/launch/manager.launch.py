"""Robot managers plus /<robot>/slam/{mapping,localizing,lifecycle_manager}."""

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
            "robot_id": ns,
            "mapper_params_file": mapper,
            "localization_params_file": localize,
            "map_file": map_file,
            "map_frame": "map",
            "odom_frame": f"{ns}/odom",
            "base_frame": f"{ns}/base_footprint",
            "scan_topic": f"/{ns}/scan_2d",
            "mapping_map_topic": f"/{ns}/mapping",
            "static_map_topic": f"/{ns}/map",
            "slam_child_namespace": f"/{ns}/slam",
            "initial_slam_mode": initial,
            "tracked_lifecycle_nodes": [
                "heartbeat",
                "navigation/lifecycle_manager",
                "map_server",
            ],
        }
    ]


def _launch_setup(context, *args, **kwargs):
    slam_params = _slam_params(context)[0]
    ns = slam_params["robot_id"]
    return [
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="localizing",
            namespace="slam",
            output="screen",
            parameters=[slam_params["localization_params_file"], {
                "use_sim_time": slam_params["use_sim_time"],
                "base_frame_id": slam_params["base_frame"],
                "odom_frame_id": slam_params["odom_frame"],
                "global_frame_id": "map",
            }],
            remappings=[
                ("scan", slam_params["scan_topic"]),
                ("map", slam_params["static_map_topic"]),
                ("initialpose", f"/{ns}/initial"),
                ("amcl_pose", f"/{ns}/amcl_pose"),
            ],
        ),
        Node(
            package="manager",
            executable="stack_lifecycle_manager_node",
            name="lifecycle_manager",
            namespace="slam",
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
                "robot_model", default_value="OP1",
                description="Robot model/profile used to select params/params/<model>/.",
            ),
            DeclareLaunchArgument(
                "localization_pose_topic",
                default_value="amcl_pose",
                description="Empty string disables pose-based ready transition.",
            ),
            DeclareLaunchArgument(
                "health_monitor_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manager"), "config", "health_monitor_op1.yaml"]
                ),
                description="YAML profile containing health_monitor ping_nodes.",
            ),
            DeclareLaunchArgument(
                "semantic_location_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manager"), "config", "semantic_location_op1.yaml"]
                ),
                description="YAML profile containing semantic_regions for the robot model.",
            ),
            DeclareLaunchArgument(
                "semantic_map_root",
                default_value="",
                description="Root containing <current_map>/<current_map>_semantic.yaml.",
            ),
            DeclareLaunchArgument(
                "require_nav2",
                default_value="false",
                description=(
                    "Compatibility switch; navigation readiness is defined by "
                    "health_monitor ping_nodes."
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
                description=(
                    "Occupancy grid yaml for map_server → /<ns>/map. "
                    "Required for initial_slam_mode:=localize."
                ),
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
                            LaunchConfiguration("health_monitor_params_file"),
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                                "required_nodes": ["heartbeat"],
                            }
                        ],
                        remappings=[("stack_lifecycle", "slam/stack_lifecycle")],
                    ),
                    Node(
                        package="manager",
                        executable="health_monitor_node",
                        name="health_monitor",
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("require_nav2")),
                        parameters=[
                            LaunchConfiguration("health_monitor_params_file"),
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                                "required_nodes": ["heartbeat"],
                            }
                        ],
                        remappings=[("stack_lifecycle", "slam/stack_lifecycle")],
                    ),
                    Node(
                        package="manager",
                        executable="semantic_location_query_node",
                        name="semantic_location_query",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("semantic_location_params_file"),
                            {
                                "robot_model": LaunchConfiguration("robot_model"),
                                "map_frame": "map",
                                "semantic_map_root": LaunchConfiguration(
                                    "semantic_map_root"
                                ),
                            },
                        ],
                    ),
                    Node(
                        package="manager",
                        executable="task_manager_node",
                        name="task_manager",
                        output="screen",
                    ),
                    Node(
                        package="relocalization",
                        executable="relocalization_node",
                        name="relocalization",
                        output="screen",
                        parameters=[
                            {
                                "map_root": LaunchConfiguration("semantic_map_root"),
                                "map_frame": "map",
                            }
                        ],
                    ),
                    OpaqueFunction(function=_launch_setup),
                ]
            ),
        ]
    )
