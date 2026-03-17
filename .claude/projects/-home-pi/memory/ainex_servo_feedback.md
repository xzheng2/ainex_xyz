# Ainex Servo Feedback

## Service
`/ros_robot_controller/bus_servo/get_state` (type: `ros_robot_controller/GetBusServoState`)

Request: array of `GetBusServoCmd` — one entry per servo ID, set flags to `1`.
Response: array of `BusServoState` — one entry per requested servo.

## Available Feedback Flags

| Flag in request | Response field | Description | Units |
|---|---|---|---|
| `get_position` | `position` | Current joint angle | raw (0–1000) |
| `get_voltage` | `voltage` | Input voltage | mV (÷1000 = V) |
| `get_temperature` | `temperature` | Internal temperature | °C |
| `get_offset` | `offset` | Trim offset | raw |
| `get_position_limit` | `position_limit` | Min/max angle limits | raw |
| `get_voltage_limit` | `voltage_limit` | Min/max voltage limits | mV |
| `get_max_temperature_limit` | `max_temperature_limit` | Over-temp cutoff | °C |
| `get_torque_state` | `enable_torque` | Torque enabled (0/1) | — |

## Example Commands

Single servo, voltage only:
```bash
rosservice call /ros_robot_controller/bus_servo/get_state "cmd: [{id: 1, get_voltage: 1}]"
```

Multiple fields at once:
```bash
rosservice call /ros_robot_controller/bus_servo/get_state "cmd: [{id: 1, get_position: 1, get_temperature: 1, get_voltage: 1}]"
```

Continuous monitoring at 10 Hz:
```bash
watch -n 0.1 'rosservice call /ros_robot_controller/bus_servo/get_state "cmd: [{id: 1, get_voltage: 1}]"'
```

All 24 servos (use -n 2 or slower to avoid RS485 overload):
```bash
watch -n 2 'rosservice call /ros_robot_controller/bus_servo/get_state "cmd: [{id: 1, get_voltage: 1},{id: 2, get_voltage: 1},{id: 3, get_voltage: 1},{id: 4, get_voltage: 1},{id: 5, get_voltage: 1},{id: 6, get_voltage: 1},{id: 7, get_voltage: 1},{id: 8, get_voltage: 1},{id: 9, get_voltage: 1},{id: 10, get_voltage: 1},{id: 11, get_voltage: 1},{id: 12, get_voltage: 1},{id: 13, get_voltage: 1},{id: 14, get_voltage: 1},{id: 15, get_voltage: 1},{id: 16, get_voltage: 1},{id: 17, get_voltage: 1},{id: 18, get_voltage: 1},{id: 19, get_voltage: 1},{id: 20, get_voltage: 1},{id: 21, get_voltage: 1},{id: 22, get_voltage: 1},{id: 23, get_voltage: 1},{id: 24, get_voltage: 1}]" | grep voltage'
```

## Bug Fixes Applied (Mar 2026)
- `ros_robot_controller_node.py` line 195: `bus_servo_read_voltage` → `bus_servo_read_vin`
- `ros_robot_controller_node.py` line 215: `bus_servo_read_torque` → `bus_servo_read_torque_state`
