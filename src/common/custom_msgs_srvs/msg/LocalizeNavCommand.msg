# Web/桥 → task_manager：仅作载荷；由桥发布到 ``/<ns>/localize_nav_command``，task_manager 订阅后顺序调用
# ``set_heartbeat_params``（task idle → current_map → robot_status=LOCALIZATION_LOST），再按需发布 ``initial``。
# ``robot_id`` 须与本机命名空间叶子一致（如 ns=/robot2 则填 robot2）。
std_msgs/Header header
string robot_id
string map_name
bool set_initial_pose
float64 x
float64 y
float64 yaw
