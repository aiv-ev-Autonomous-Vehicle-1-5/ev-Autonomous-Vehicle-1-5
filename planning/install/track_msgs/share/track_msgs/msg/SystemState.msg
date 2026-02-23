# System state codes
uint8 OFFLINE=0
uint8 MANUAL=1
uint8 AUTO_STANDBY=2
uint8 AUTO_ACTIVE=3
uint8 AUTO_HOLD=4
uint8 STATE_INFEASIBLE=5
uint8 EMERGENCY_STOP=6

std_msgs/Header header
uint8 state
