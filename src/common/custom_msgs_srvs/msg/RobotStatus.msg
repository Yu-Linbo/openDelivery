std_msgs/Header header
string robot_name
# Robot model/profile identifier, e.g. OP1. It selects params/params/<model>/ at bringup.
string robot_model
string current_map
# Semantic position summary: "current_position;next_position:distance;...;".
string current_position

# --- robot_status: 上线生命周期（health_monitor → set_heartbeat_params → heartbeat）---
# echo 直接显示下列字面量；常量供代码引用，避免魔法字符串
string ROBOT_STATUS_INITIALIZING=initializing
string ROBOT_STATUS_LOCALIZING=localizing
string ROBOT_STATUS_LOCALIZATION_LOST=localization_lost
string ROBOT_STATUS_READY=ready
string ROBOT_STATUS_SHUTDOWN=shutdown
string robot_status

# --- task_status: 作业状态（task_manager → set_heartbeat_params → heartbeat）---
string TASK_STATUS_IDLE=idle
string TASK_STATUS_MAPPING=mapping
string TASK_STATUS_DELIVERY=delivery
string TASK_STATUS_CLEANING=cleaning
string TASK_STATUS_PATROLLING=patrolling
string task_status

# true: 来自仿真栈；false: 真机或未知（旧节点未填时桥接侧按 false 处理）
bool is_simulation

# --- task_progress: 当前任务进度（task_manager / 外部写入 set_heartbeat_params）---
# 范围 0.0-1.0；-1.0 表示不适用（无任务或未支持）
float32 task_progress
