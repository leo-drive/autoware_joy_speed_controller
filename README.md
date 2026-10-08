# # autoware_joy_speed_controller

## Role

`autoware_joy_speed_controller` is the package to convert a joy msg to autoware commands (e.g. steering wheel, speed control, shift, turn signal, engage) for a vehicle.

## Usage

### ROS 2 launch

```bash
# With default config (ds4)
ros2 launch autoware_joy_speed_controller joy_controller.launch.xml

# Default config but select from the existing parameter files
ros2 launch autoware_joy_speed_controller joy_controller_param_selection.launch.xml joy_type:=ds4 # or g29, p65, xbox

# Override the param file
ros2 launch autoware_joy_speed_controller joy_controller.launch.xml config_file:=/path/to/your/param.yaml
```

## Input / Output

### Input topics

| Name               | Type                    | Description                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | joy controller command            |


### Output topics

| Name                                | Type                                                | Description                              |
| ----------------------------------- | --------------------------------------------------- | ---------------------------------------- |
| `~/control/command/control_cmd`          | autoware_control_msgs::msg::Control                 | lateral and longitudinal control command |
| `~/control/command/gear_cmd`                    | autoware_vehicle_msgs::msg::GearCommand>     | gear command                             |
| `~/control/command/turn_indicators_cmd`              | autoware_vehicle_msgs::msg::TurnIndicatorsCommand    | turn signal command                      |
| `~/control/command/hazard_lights_cmd` | autoware_vehicle_msgs::msg::HazardLightsCommand| hazard light command
| `~/control/current_gate_mode`                | tier4_control_msgs::msg::GateMode                   | gate mode (Auto or External)             |
| `~/output/heartbeat`                | tier4_external_api_msgs::msg::Heartbeat             | heartbeat                                |
| `~/output/vehicle_engage`           | autoware_vehicle_msgs::msg::Engage                  | vehicle engage                           |
| `~/control/command/emergency_cmd` | tier4_vehicle_msgs::msg::VehicleEmergencyStamped | emergency command  |
| `~/autoware/engage` | autoware_vehicle_msgs::msg::Engage | autoware engage command |

## Parameters

| Parameter                 | Type   | Description                                                                                                        |
| ------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------ |
| `joy_type`                | string | Type of joystick controller (default: DS4)                                                                        |
| `update_rate`             | double | Frequency (Hz) at which joystick input is processed and the command is updated                                    |
| `publish_rate`            | double | Frequency (Hz) at which control and emergency commands are published (default: 33.0, must match the vehicle interface) |
| `steer_ratio`             | double | Ratio used to scale steering input to achieve the desired steering response                                       |
| `steer_rate`              | double | Maximum steering command speed (rad/s)                                                                             |
| `steer_expo`              | double | Stick curve, 0 = linear, 1 = cubic; higher gives finer control near center (default: 0.5)                         |
| `steer_time_constant`     | double | Low-pass time constant (s) applied to the stick before the rate limit (default: 0.15)                             |
| `steering_angle_velocity` | double | Maximum rate of change for the steering angle                                                                     |
| `require_deadman`         | bool   | Command zero velocity unless the dead man's switch is held (DS4: R2) (default: true)                               |
| `velocity_gain`           | double | Target velocity increase (m/s) per second of holding the accelerate button                                         |
| `brake_gain`              | double | Target velocity decrease (m/s) per second of holding the brake button (default: 0.75)                             |
| `max_velocity`            | double | Maximum allowable velocity in both forward and reverse directions                                                  |
| `accel_smooth_factor`     | double | Smoothing factor applied to acceleration for smoother transitions                                                  |
| `decel_smooth_factor`     | double | Smoothing factor applied to deceleration for smoother transitions                                                  |



## DS4 Joystick Key Map

| Action               | Button                     |
| -------------------- | -------------------------- |
| Dead man's switch    | R2 (hold to move)          |
| Decrease Speed       |  ×                         |
| Increase Speed       |  □                         |
| Steering             | Left Stick Left Right      |
| Shift up             | Cursor Up                  |
| Shift down           | Cursor Down                |
| Shift Drive          | Cursor Right               |
| Shift Reverse        | Cursor Left                |
| Turn Signal Left     | L1                         |
| Turn Signal Right    | R1                         |
| Clear Turn Signal    | SHARE                      |
| Gate Mode            | OPTIONS                    |
| Emergency Stop       | PS                         |
| Clear Emergency Stop | SHARE + PS                 |
| Autoware Engage      | ○                          |
| Autoware Disengage   | SHARE + ○                  |
| Vehicle Engage       | △                          |
| Vehicle Disengage    | SHARE + △                  |

Buttons act once per press. Releasing R2 drops the target velocity to zero, and so does any gear change.

## Remote Driving over MQTT

The joystick can be plugged into any computer with a browser; its state reaches
the vehicle through an MQTT broker. The speed controller and all safety checks
keep running on the vehicle.

```text
DS4 -> browser (web/operator.html) -> MQTT broker -> cloud_joy_bridge -> /joy -> joy_speed_controller -> vehicle interface
```

### Broker requirements

- A TLS listener for the vehicle (MQTT, e.g. port 8883) and a secure WebSocket
  listener for the browser (WSS).
- Authentication, and an ACL that allows only the operator account to publish to
  the joystick topic. Anyone who can publish there can drive the vehicle.

### Vehicle

```bash
sudo apt install python3-paho-mqtt
# set mqtt.host and mqtt.topic in config/cloud_joy_bridge.param.yaml (or a copy of it)
export JOY_MQTT_PASSWORD='...'
ros2 launch autoware_joy_speed_controller joy_controller.launch.xml \
  input_source:=cloud cloud_bridge_config_file:=/path/to/cloud_joy_bridge.param.yaml
```

### Operator

Serve the page from localhost (the Gamepad API needs a secure context) and open
it in Chrome or Edge:

```bash
cd web && python3 -m http.server 8000
# open http://localhost:8000/operator.html, enter the wss:// broker URL, topic and credentials
```

### Link loss

| Time without operator input | Vehicle behavior                                                        |
| --------------------------- | ----------------------------------------------------------------------- |
| up to `link_timeout` (0.5 s) | latest operator state is used                                           |
| up to `stop_publish_after` (3 s) | neutral state (R2 released): zero velocity command, controlled stop |
| longer                      | `/joy` stops; the vehicle interface falls back to PARK + emergency       |

The page stops sending when its tab is hidden. Only one operator page is
accepted at a time; another page can take over once the active one has been
silent for `link_timeout`. Retained and out-of-order messages are dropped.
