std_msgs/Header header

string task_id

string OPERATION_CALL=call
string OPERATION_RIDE=ride
string operation

string from_floor
string target_floor

# Pre-switch map pose used for Relocalize MODE_POSE_FIRST after loading target_floor.
geometry_msgs/PoseWithCovarianceStamped relocalization_pose
bool use_pose_first

# Gazebo destination in the target map/world coordinate convention.
geometry_msgs/Pose target_inside_pose
