std_msgs/Header header

# Stable task identifier assigned by the scheduling side.
string task_id

string TASK_TYPE_NAVIGATION=navigation
string TASK_TYPE_PATROL=patrol
string TASK_TYPE_FOLLOWING=following
string task_type

# Display/configuration name. Following tasks may use it to identify an offline path JSON.
string task_name

string END_ACTION_WAITING=waiting
string END_ACTION_BACK=back
string end_action

# floor_ids is empty or has exactly the same length as poses.
string[] floor_ids
geometry_msgs/Pose[] poses
