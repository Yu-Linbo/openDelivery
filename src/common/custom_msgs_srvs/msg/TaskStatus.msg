std_msgs/Header header

string task_id

string STATUS_WAITING=Waiting
string STATUS_NAVIGATING=Navigating
string STATUS_PATROLLING=Patrolling
string STATUS_FOLLOWING=Following
string STATUS_CLEANING=Cleaning
string STATUS_CALLING_ELEVATOR=CallingElevator
string STATUS_ELEVATOR_ENTERING=ElevatorEntering
string STATUS_RIDING_ELEVATOR=RidingElevator
string STATUS_MOVING_MODEL=MovingModel
string STATUS_SWITCHING_MAP=SwitchingMap
string STATUS_RELOCALIZING=Relocalizing
string STATUS_FINISHED=Finished
string STATUS_FAILED=Failed
string STATUS_TERMINATED=Terminated
string STATUS_PAUSED=Paused

# Ordered execution units and their current status. Arrays have equal lengths.
string[] work_queue
string[] model_status
string task_status
string message
uint32 current_index
uint32 total_count
