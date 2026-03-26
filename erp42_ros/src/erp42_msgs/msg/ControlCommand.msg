# erp42_msgs/msg/ControlCommand.msg

# Message header (센서 타임스탬프 전파 → ros2 topic delay 측정용)
std_msgs/Header header

# Vehicle speed (m/s)
float64 speed

# Steering angle (rad)
float64 steering

# Brake
# ERP42 (0 - 150)
uint8 brake