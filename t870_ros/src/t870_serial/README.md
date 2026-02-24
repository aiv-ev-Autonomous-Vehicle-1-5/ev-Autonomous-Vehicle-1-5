# t870_serial
Henes T870 serial communication package

<br/>

## serial_bridge
ROS2 communication interface and Henes T870 PCU serial packet conversion node 

```bash
$ ros2 launch t870_serial serial_bridge.launch.py
```

### Topic / Service Names
| Interface | Entitiy      | Type                             | Name                      | Description                                      |
| --------- | ------------ | -------------------------------- | ------------------------- | ------------------------------------------------ |
| Topic     | Subscription | **t870_msgs/msg/ControlCommand** | **/t870/control_command** | Control command includes speed, steering         |
| Topic     | Publisher    | **t870_msgs/msg/Feedback**       | **/t870/feedback**        | Feedback from Henes T870                         |
| Servie    | Server       | **t870_msgs/srv/ModeCommand**    | **/t870/mode_command**    | Mode command includes control mode, E-stop, gear |

### QoS
The ModeCommand.srv service QoS profile is the system default.
| QoS Policy  | QoS Policy Key |
| ----------- | -------------- |
| History     | **Keep Last**  |
| Depth       | **1**          |
| Reliability | **Reliable**   |
| Durability  | **Volatile**   |

### Parameters
| Parameter Name          | Unit | Description                                                                                           |
| ----------------------- | ---- | ----------------------------------------------------------------------------------------------------- |
| **port_path**           | -    | Serial port path ( e.g. /dev/ttyUSB0 )                                                                |
| **baud_rate**           | -    | Serial communication speed. only 115200 or 9600 can be selected.                                      |
| **max_speed_mps**       | m/s  | The vehicle's maximum linear speed. When using Auto mode, the PCU limits it to 1.60 m/s.              |
| **max_steering_deg**    | deg  | The vehicle's maximum steering angle. Due to hardware limitations, 20 degrees or less is recommended. |
| **steering_offset_deg** | deg  | Provides an offset to the steering angle. Left is positive (+), right is negative (-).                |
