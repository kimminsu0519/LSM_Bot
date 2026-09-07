std_msgs/Header header
string robot_id
uint8 current_state
geometry_msgs/Pose pose
float32 battery_level
string current_task_id

uint8 STATE_CHARGING=0
uint8 STATE_MOVING_UNLOADED=1
uint8 STATE_WORKING_LOADING=2
uint8 STATE_MOVING_LOADED=3
uint8 STATE_WORKING_UNLOADING=4
