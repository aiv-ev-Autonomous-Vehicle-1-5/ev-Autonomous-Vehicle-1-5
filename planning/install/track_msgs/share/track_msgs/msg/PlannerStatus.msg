# track_msgs/msg/PlannerStatus.msg
# Planning pipeline status output

std_msgs/Header header

uint8 OK          = 0
uint8 STOP        = 1
uint8 INFEASIBLE  = 2
uint8 STALE       = 3
uint8 status

string reason                       # Human-readable reason code
