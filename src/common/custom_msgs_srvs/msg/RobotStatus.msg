std_msgs/Header header
string robot_name
string current_map

# --- robot_status: 上线生命周期（health_monitor → set_heartbeat_params → heartbeat）---
uint8 ROBOT_STATUS_INITIALIZING=0
uint8 ROBOT_STATUS_LOCALIZING=1
uint8 ROBOT_STATUS_LOCALIZATION_LOST=2
uint8 ROBOT_STATUS_READY=3
uint8 ROBOT_STATUS_SHUTDOWN=4
uint8 robot_status

# --- task_status: 作业状态（task_manager → set_heartbeat_params → heartbeat）---
uint8 TASK_STATUS_IDLE=0
uint8 TASK_STATUS_MAPPING=1
uint8 TASK_STATUS_DELIVERY=2
uint8 TASK_STATUS_CLEANING=3
uint8 TASK_STATUS_PATROLLING=4
uint8 task_status

# true: 来自仿真栈；false: 真机或未知（旧节点未填时桥接侧按 false 处理）
bool is_simulation
