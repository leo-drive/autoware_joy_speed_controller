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
| `update_rate`             | double | Frequency (Hz) at which control commands are published                                                            |
| `steer_ratio`             | double | Ratio used to scale steering input to achieve the desired steering response                                       |
| `steer_rate`              | double | Rate at which the steering ratio is applied to achieve the required steering response                            |
| `steering_angle_velocity` | double | Maximum rate of change for the steering angle                                                                     |
| `velocity_gain`           | double | Gain factor used to convert acceleration input to velocity                                                         |
| `max_velocity`            | double | Maximum allowable velocity in both forward and reverse directions                                                  |
| `accel_smooth_factor`     | double | Smoothing factor applied to acceleration for smoother transitions                                                  |
| `decel_smooth_factor`     | double | Smoothing factor applied to deceleration for smoother transitions                                                  |



## DS4 Joystick Key Map

| Action               | Button                     |
| -------------------- | -------------------------- |
| Decrease Speed       |  ×                         |
| Increase Speed       |  □                         |
| Steering             | Left Stick Left Right      |
| Shift up             | Cursor Up                  |
| Shift down           | Cursor Down                |
| Shift Drive          | Cursor Left                |
| Shift Reverse        | Cursor Right               |
| Turn Signal Left     | L1                         |
| Turn Signal Right    | R1                         |
| Clear Turn Signal    | SHARE                      |
| Gate Mode            | OPTIONS                    |
| Emergency Stop       | PS                         |
| Clear Emergency Stop | PS                         |
| Autoware Engage      | ○                          |
| Autoware Disengage   | ○                          |
| Vehicle Engage       | △                          |
| Vehicle Disengage    | △                          |
