# Planner status codes
uint8 OK=0
uint8 STOP=1
uint8 INFEASIBLE=2
uint8 STALE=3

std_msgs/Header header
uint8 status
string reason
